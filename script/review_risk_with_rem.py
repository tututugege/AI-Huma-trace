#!/usr/bin/env python3
"""Use the Rem review AI to inspect a PR diff or pushed commits."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


API_URL = "https://api.sophnet.com/v1/chat/completions"
MODEL = "DeepSeek-V4-Flash-0731"
MAX_DIFF_CHARS = 240_000
RISK_RE = re.compile(r"^RISK_LEVEL:\s*(LOW|MEDIUM|HIGH|CRITICAL)\s*$", re.MULTILINE)
ZERO_SHA = "0" * 40


def git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        errors="replace",
    )
    return result.stdout


def shorten_diff(diff: str) -> tuple[str, bool]:
    if len(diff) <= MAX_DIFF_CHARS:
        return diff, False
    omitted = len(diff) - MAX_DIFF_CHARS
    marker = (
        "\n\n[DIFF TRUNCATED BY REVIEWER: "
        f"{omitted} characters omitted because the change is too large]\n"
    )
    # Keep both the beginning and the end: declarations tend to be near the
    # beginning while tests and workflow changes are often near the end.
    head_size = (MAX_DIFF_CHARS - len(marker)) * 2 // 3
    tail_size = MAX_DIFF_CHARS - len(marker) - head_size
    return diff[:head_size] + marker + diff[-tail_size:], True


def call_rem(api_key: str, system_prompt: str, subject: str, diff: str) -> str:
    user_prompt = (
        f"审查对象：{subject}\n\n"
        "以下是需要审查的 git diff。只分析改动引入的风险。\n"
        "----- BEGIN UNTRUSTED DIFF -----\n"
        f"{diff}\n"
        "----- END UNTRUSTED DIFF -----\n"
    )
    payload = json.dumps(
        {
            "model": MODEL,
            "max_tokens": 8192,
            "temperature": 0.1,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt},
            ],
        },
        ensure_ascii=False,
    ).encode("utf-8")
    request = urllib.request.Request(
        API_URL,
        data=payload,
        method="POST",
        headers={
            "Authorization": f"Bearer {api_key}",
            "Content-Type": "application/json",
            "User-Agent": "qm1-rem-risk-review/1.0",
        },
    )
    try:
        with urllib.request.urlopen(request, timeout=300) as response:
            data = json.load(response)
    except urllib.error.HTTPError as exc:
        detail = exc.read(4096).decode("utf-8", errors="replace")
        raise RuntimeError(f"Rem API HTTP {exc.code}: {detail}") from exc
    except urllib.error.URLError as exc:
        raise RuntimeError(f"Rem API request failed: {exc.reason}") from exc

    try:
        message = data["choices"][0]["message"]
        content = message.get("content") or message.get("reasoning_content")
    except (KeyError, IndexError, TypeError, AttributeError) as exc:
        raise RuntimeError("Rem API returned an unexpected response") from exc
    if not isinstance(content, str) or not content.strip():
        raise RuntimeError("Rem API returned an empty review")
    return content.strip()


def risk_level(review: str) -> str:
    match = RISK_RE.search(review)
    return match.group(1) if match else "UNKNOWN"


def review_one(
    api_key: str, system_prompt: str, sha: str, subject: str, diff: str
) -> dict[str, Any]:
    compact_diff, truncated = shorten_diff(diff)
    if not compact_diff.strip():
        return {
            "sha": sha,
            "subject": subject,
            "status": "skipped",
            "risk_level": "LOW",
            "truncated": False,
            "body": "RISK_LEVEL: LOW\n\n该改动没有可审查的文本 diff。",
        }
    try:
        body = call_rem(api_key, system_prompt, subject, compact_diff)
        return {
            "sha": sha,
            "subject": subject,
            "status": "ok",
            "risk_level": risk_level(body),
            "truncated": truncated,
            "body": body,
        }
    except Exception as exc:  # Keep review advisory; never block normal CI.
        return {
            "sha": sha,
            "subject": subject,
            "status": "error",
            "risk_level": "UNKNOWN",
            "truncated": truncated,
            "body": f"风险审查服务调用失败：{exc}",
        }


def commit_list(before: str, after: str) -> list[str]:
    if not before or before == ZERO_SHA:
        return [after]
    try:
        commits = git("rev-list", "--reverse", f"{before}..{after}").splitlines()
    except subprocess.CalledProcessError:
        commits = []
    return commits or [after]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=("pr", "push"))
    parser.add_argument("--prompt-file", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--base")
    parser.add_argument("--head")
    parser.add_argument("--before")
    parser.add_argument("--after")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    output_path = Path(args.output)
    api_key = os.environ.get("REM_API_KEY", "").strip()
    if not api_key:
        output_path.write_text(
            json.dumps(
                {
                    "mode": args.mode,
                    "status": "skipped",
                    "message": "GitHub Secret REM_API_KEY is not configured",
                    "reviews": [],
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
        print("::warning::REM_API_KEY is not configured; Rem risk review skipped")
        return 0

    system_prompt = Path(args.prompt_file).read_text(encoding="utf-8")
    reviews: list[dict[str, Any]] = []
    if args.mode == "pr":
        if not args.base or not args.head:
            raise SystemExit("pr mode requires --base and --head")
        diff = git(
            "diff",
            "--no-ext-diff",
            "--find-renames",
            "--find-copies",
            "--unified=80",
            f"{args.base}...{args.head}",
            "--",
        )
        reviews.append(
            review_one(
                api_key,
                system_prompt,
                args.head,
                f"PR diff {args.base[:12]}...{args.head[:12]}",
                diff,
            )
        )
    else:
        if not args.after:
            raise SystemExit("push mode requires --after")
        for sha in commit_list(args.before or "", args.after):
            subject = git("show", "-s", "--format=%h %s", sha).strip()
            diff = git(
                "show",
                "--no-ext-diff",
                "--find-renames",
                "--find-copies",
                "--format=fuller",
                "--unified=80",
                sha,
                "--",
            )
            reviews.append(review_one(api_key, system_prompt, sha, subject, diff))

    result = {
        "mode": args.mode,
        "status": "ok" if all(r["status"] != "error" for r in reviews) else "partial",
        "reviews": reviews,
    }
    output_path.write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    for review in reviews:
        level = review["risk_level"]
        if level in ("HIGH", "CRITICAL"):
            print(f"::warning title=Rem risk review::{review['sha'][:12]}: {level}")
        elif review["status"] == "error":
            print(f"::warning title=Rem review unavailable::{review['sha'][:12]}")
        else:
            print(f"{review['sha'][:12]}: {level}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
