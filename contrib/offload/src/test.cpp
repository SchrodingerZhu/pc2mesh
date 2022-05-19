#include <offload/test.h>
#include <cmath>
namespace offload {
    float test(float* a, float* b) {
#pragma omp target map(tofrom: b[0:1024]) map(to: a[0:1024])
#pragma omp teams distribute parallel for default(none) shared(a, b)
        for (int i = 0; i < 1024; ++i) {
            if (i & 1) {
                b[i] = ::cosf(a[i] * a[i] + b[i] * b[i]);
            } else {
                b[i] = ::tanf(a[i] * a[i] + b[i] * b[i]);
            }
        }
        float x = 0;
#pragma omp target map(tofrom: x) map(to: b[0:1024])
#pragma omp teams distribute reduction(+:x) default(none) shared(b)
        for (int i = 0; i < 1024; ++i) {
            x += b[i];
        }
        return x;
    }
}