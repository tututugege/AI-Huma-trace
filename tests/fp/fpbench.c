// Small FP kernels for the register-file port study (EXP-032): KERNEL selects one; compile with -O2 -ffp-contract=fast for rv32/rv64 imafd.
//   0 dgemm 24x24 (double, 4 independent accumulators)   1 saxpy + dot (float streams)   2 jacobi 2D stencil (float)
//   3 n-body style (double, fsqrt + fdiv)                2 FIR filter (float)           5 mixed int/FP (conversions, compares)
__asm__(".globl main\nmain:\n li t0, 0x2000\n csrs mstatus, t0\n j fp_main\n");
#define N 24
static double A[N][N], B[N][N], C[N][N];
static float X[1024], Y[1024], H[16], Z[1024];
static float G[48][48], G2[48][48];
static volatile double sink;
static double PX[48], PY[48], PZ[48], FX[48], FY[48], FZ[48], M[48];

int fp_main(void) {
  double acc = 0;
  float facc = 0;
  long iacc = 0;
  for (int i = 0; i < 1024; i++) { X[i] = (float)(i % 17) * 0.5f + 1.0f; Y[i] = (float)(i % 13) * 0.25f; }
  for (int i = 0; i < 16; i++) H[i] = 1.0f / (float)(i + 1);
  for (int i = 0; i < N; i++) for (int j = 0; j < N; j++) { A[i][j] = (i * 3 + j) * 0.01; B[i][j] = (i - j) * 0.02; }
  for (int i = 0; i < 48; i++) { PX[i] = i * 0.7; PY[i] = i * 1.3 + 2; PZ[i] = i * 0.9 - 1; M[i] = 1.0 + i * 0.1; for (int j = 0; j < 48; j++) G[i][j] = (i ^ j) * 0.125f; }
#if KERNEL == 0
  for (int rep = 0; rep < REPS; rep++)
    for (int i = 0; i < N; i++)
      for (int j = 0; j < N; j++) {
        double s0 = 0, s1 = 0, s2 = 0, s3 = 0;
        for (int k = 0; k < N; k += 4) { s0 += A[i][k] * B[k][j]; s1 += A[i][k + 1] * B[k + 1][j]; s2 += A[i][k + 2] * B[k + 2][j]; s3 += A[i][k + 3] * B[k + 3][j]; }
        C[i][j] = s0 + s1 + s2 + s3;
        acc += C[i][j];
      }
#elif KERNEL == 1
  for (int rep = 0; rep < REPS; rep++) {
    for (int i = 0; i < 1024; i++) Z[i] = 2.5f * X[i] + Y[i];
    float d0 = 0, d1 = 0, d2 = 0, d3 = 0;
    for (int i = 0; i < 1024; i += 4) { d0 += Z[i] * X[i]; d1 += Z[i + 1] * X[i + 1]; d2 += Z[i + 2] * X[i + 2]; d3 += Z[i + 3] * X[i + 3]; }
    facc += d0 + d1 + d2 + d3;
  }
#elif KERNEL == 2
  for (int rep = 0; rep < REPS; rep++) {
    for (int i = 1; i < 47; i++) for (int j = 1; j < 47; j++) G2[i][j] = 0.2f * (G[i][j] + G[i - 1][j] + G[i + 1][j] + G[i][j - 1] + G[i][j + 1]);
    for (int i = 1; i < 47; i++) for (int j = 1; j < 47; j++) G[i][j] = G2[i][j];
  }
  facc = G[20][20];
#elif KERNEL == 3
  for (int rep = 0; rep < REPS; rep++) {
    for (int i = 0; i < 48; i++) { FX[i] = FY[i] = FZ[i] = 0; }
    for (int i = 0; i < 24; i++)
      for (int j = 0; j < 48; j++) {
        double dx = PX[j] - PX[i], dy = PY[j] - PY[i], dz = PZ[j] - PZ[i];
        double r2 = dx * dx + dy * dy + dz * dz + 0.01;
        double inv = 1.0 / __builtin_sqrt(r2);
        double f = M[j] * inv * inv * inv;
        FX[i] += f * dx; FY[i] += f * dy; FZ[i] += f * dz;
      }
    acc += FX[3] + FY[5] + FZ[7];
  }
#elif KERNEL == 4
  for (int rep = 0; rep < REPS; rep++)
    for (int i = 0; i < 1000; i++) { float s = 0; for (int k = 0; k < 16; k++) s += H[k] * X[i + k]; Z[i] = s; }
  facc = Z[10];
#else
  for (int rep = 0; rep < REPS; rep++)
    for (int i = 0; i < 1000; i++) { int v = (int)(X[i] * 3.0f); iacc += v & 7; if (Y[i] > 1.0f) iacc += (long)(Y[i] * 2.0f); facc += (float)(iacc & 255); }
#endif
  sink = acc + (double)facc + (double)iacc;
  return 0;
}
