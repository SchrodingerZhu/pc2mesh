#include <cmath>

namespace tomasakeninemoeller {
    __attribute__((always_inline)) static inline int
    coplanar_tri_tri(const double N[3], const double V0[3], const double V1[3], const double V2[3],
                     const double U0[3], const double U1[3], const double U2[3]) {
        double A[3];
        short i0, i1;


        A[0] = (double(fabs(N[0])));
        A[1] = (double(fabs(N[1])));
        A[2] = (double(fabs(N[2])));
        if (A[0] > A[1]) {
            if (A[0] > A[2]) {
                i0 = 1;
                i1 = 2;
            } else {
                i0 = 0;
                i1 = 1;
            }
        } else {
            if (A[2] > A[1]) {
                i0 = 0;
                i1 = 1;
            } else {
                i0 = 0;
                i1 = 2;
            }
        }


        {
            double Ax, Ay, Bx, By, Cx, Cy, e, d, f;
            Ax = V1[i0] - V0[i0];
            Ay = V1[i1] - V0[i1];
            Bx = U0[i0] - U1[i0];
            By = U0[i1] - U1[i1];
            Cx = V0[i0] - U0[i0];
            Cy = V0[i1] - U0[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U1[i0] - U2[i0];
            By = U1[i1] - U2[i1];
            Cx = V0[i0] - U1[i0];
            Cy = V0[i1] - U1[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U2[i0] - U0[i0];
            By = U2[i1] - U0[i1];
            Cx = V0[i0] - U2[i0];
            Cy = V0[i1] - U2[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
        };
        {
            double Ax, Ay, Bx, By, Cx, Cy, e, d, f;
            Ax = V2[i0] - V1[i0];
            Ay = V2[i1] - V1[i1];
            Bx = U0[i0] - U1[i0];
            By = U0[i1] - U1[i1];
            Cx = V1[i0] - U0[i0];
            Cy = V1[i1] - U0[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U1[i0] - U2[i0];
            By = U1[i1] - U2[i1];
            Cx = V1[i0] - U1[i0];
            Cy = V1[i1] - U1[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U2[i0] - U0[i0];
            By = U2[i1] - U0[i1];
            Cx = V1[i0] - U2[i0];
            Cy = V1[i1] - U2[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
        };
        {
            double Ax, Ay, Bx, By, Cx, Cy, e, d, f;
            Ax = V0[i0] - V2[i0];
            Ay = V0[i1] - V2[i1];
            Bx = U0[i0] - U1[i0];
            By = U0[i1] - U1[i1];
            Cx = V2[i0] - U0[i0];
            Cy = V2[i1] - U0[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U1[i0] - U2[i0];
            By = U1[i1] - U2[i1];
            Cx = V2[i0] - U1[i0];
            Cy = V2[i1] - U1[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
            Bx = U2[i0] - U0[i0];
            By = U2[i1] - U0[i1];
            Cx = V2[i0] - U2[i0];
            Cy = V2[i1] - U2[i1];
            f = Ay * Bx - Ax * By;
            d = By * Cx - Bx * Cy;
            if ((f > 0 && d >= 0 && d <= f) || (f < 0 && d <= 0 && d >= f)) {
                e = Ax * Cy - Ay * Cx;
                if (f > 0) { if (e >= 0 && e <= f) return 1; } else { if (e <= 0 && e >= f) return 1; }
            };
        };


        {
            double a, b, c, d0, d1, d2;
            a = U1[i1] - U0[i1];
            b = -(U1[i0] - U0[i0]);
            c = -a * U0[i0] - b * U0[i1];
            d0 = a * V0[i0] + b * V0[i1] + c;
            a = U2[i1] - U1[i1];
            b = -(U2[i0] - U1[i0]);
            c = -a * U1[i0] - b * U1[i1];
            d1 = a * V0[i0] + b * V0[i1] + c;
            a = U0[i1] - U2[i1];
            b = -(U0[i0] - U2[i0]);
            c = -a * U2[i0] - b * U2[i1];
            d2 = a * V0[i0] + b * V0[i1] + c;
            if (d0 * d1 > 0.0) { if (d0 * d2 > 0.0) return 1; }
        };
        {
            double a, b, c, d0, d1, d2;
            a = V1[i1] - V0[i1];
            b = -(V1[i0] - V0[i0]);
            c = -a * V0[i0] - b * V0[i1];
            d0 = a * U0[i0] + b * U0[i1] + c;
            a = V2[i1] - V1[i1];
            b = -(V2[i0] - V1[i0]);
            c = -a * V1[i0] - b * V1[i1];
            d1 = a * U0[i0] + b * U0[i1] + c;
            a = V0[i1] - V2[i1];
            b = -(V0[i0] - V2[i0]);
            c = -a * V2[i0] - b * V2[i1];
            d2 = a * U0[i0] + b * U0[i1] + c;
            if (d0 * d1 > 0.0) { if (d0 * d2 > 0.0) return 1; }
        };

        return 0;
    }

    __attribute__((always_inline)) static inline int NoDivTriTriIsect(
            const double V0[3],
            const double V1[3],
            const double V2[3],
            const double U0[3],
            const double U1[3],
            const double U2[3]) {
        double E1[3], E2[3];
        double N1[3], N2[3], d1, d2;
        double du0, du1, du2, dv0, dv1, dv2;
        double D[3];
        double isect1[2], isect2[2];
        double du0du1, du0du2, dv0dv1, dv0dv2;
        short index;
        double vp0, vp1, vp2;
        double up0, up1, up2;
        double bb, cc, max;


        {
            E1[0] = V1[0] - V0[0];
            E1[1] = V1[1] - V0[1];
            E1[2] = V1[2] - V0[2];
        };
        {
            E2[0] = V2[0] - V0[0];
            E2[1] = V2[1] - V0[1];
            E2[2] = V2[2] - V0[2];
        };
        {
            N1[0] = E1[1] * E2[2] - E1[2] * E2[1];
            N1[1] = E1[2] * E2[0] - E1[0] * E2[2];
            N1[2] = E1[0] * E2[1] - E1[1] * E2[0];
        };
        d1 = -(N1[0] * V0[0] + N1[1] * V0[1] + N1[2] * V0[2]);


        du0 = (N1[0] * U0[0] + N1[1] * U0[1] + N1[2] * U0[2]) + d1;
        du1 = (N1[0] * U1[0] + N1[1] * U1[1] + N1[2] * U1[2]) + d1;
        du2 = (N1[0] * U2[0] + N1[1] * U2[1] + N1[2] * U2[2]) + d1;


        if ((double(fabs(du0))) < 0.000001) du0 = 0.0;
        if ((double(fabs(du1))) < 0.000001) du1 = 0.0;
        if ((double(fabs(du2))) < 0.000001) du2 = 0.0;

        du0du1 = du0 * du1;
        du0du2 = du0 * du2;

        if (du0du1 > 0.0f && du0du2 > 0.0f)
            return 0;


        {
            E1[0] = U1[0] - U0[0];
            E1[1] = U1[1] - U0[1];
            E1[2] = U1[2] - U0[2];
        };
        {
            E2[0] = U2[0] - U0[0];
            E2[1] = U2[1] - U0[1];
            E2[2] = U2[2] - U0[2];
        };
        {
            N2[0] = E1[1] * E2[2] - E1[2] * E2[1];
            N2[1] = E1[2] * E2[0] - E1[0] * E2[2];
            N2[2] = E1[0] * E2[1] - E1[1] * E2[0];
        };
        d2 = -(N2[0] * U0[0] + N2[1] * U0[1] + N2[2] * U0[2]);


        dv0 = (N2[0] * V0[0] + N2[1] * V0[1] + N2[2] * V0[2]) + d2;
        dv1 = (N2[0] * V1[0] + N2[1] * V1[1] + N2[2] * V1[2]) + d2;
        dv2 = (N2[0] * V2[0] + N2[1] * V2[1] + N2[2] * V2[2]) + d2;


        if ((double(fabs(dv0))) < 0.000001) dv0 = 0.0;
        if ((double(fabs(dv1))) < 0.000001) dv1 = 0.0;
        if ((double(fabs(dv2))) < 0.000001) dv2 = 0.0;


        dv0dv1 = dv0 * dv1;
        dv0dv2 = dv0 * dv2;

        if (dv0dv1 > 0.0f && dv0dv2 > 0.0f)
            return 0;


        {
            D[0] = N1[1] * N2[2] - N1[2] * N2[1];
            D[1] = N1[2] * N2[0] - N1[0] * N2[2];
            D[2] = N1[0] * N2[1] - N1[1] * N2[0];
        };


        max = (double) (double(fabs(D[0])));
        index = 0;
        bb = (double) (double(fabs(D[1])));
        cc = (double) (double(fabs(D[2])));
        if (bb > max) max = bb, index = 1;
        if (cc > max) max = cc, index = 2;


        vp0 = V0[index];
        vp1 = V1[index];
        vp2 = V2[index];

        up0 = U0[index];
        up1 = U1[index];
        up2 = U2[index];


        double a, b, c, x0, x1;
        {
            if (dv0dv1 > 0.0f) {
                a = vp2;
                b = (vp0 - vp2) * dv2;
                c = (vp1 - vp2) * dv2;
                x0 = dv2 - dv0;
                x1 = dv2 - dv1;
            } else if (dv0dv2 > 0.0f) {
                a = vp1;
                b = (vp0 - vp1) * dv1;
                c = (vp2 - vp1) * dv1;
                x0 = dv1 - dv0;
                x1 = dv1 - dv2;
            } else if (dv1 * dv2 > 0.0f || dv0 != 0.0f) {
                a = vp0;
                b = (vp1 - vp0) * dv0;
                c = (vp2 - vp0) * dv0;
                x0 = dv0 - dv1;
                x1 = dv0 - dv2;
            } else if (dv1 != 0.0f) {
                a = vp1;
                b = (vp0 - vp1) * dv1;
                c = (vp2 - vp1) * dv1;
                x0 = dv1 - dv0;
                x1 = dv1 - dv2;
            } else if (dv2 != 0.0f) {
                a = vp2;
                b = (vp0 - vp2) * dv2;
                c = (vp1 - vp2) * dv2;
                x0 = dv2 - dv0;
                x1 = dv2 - dv1;
            } else { return coplanar_tri_tri(N1, V0, V1, V2, U0, U1, U2); }
        };


        double d, e, f, y0, y1;
        {
            if (du0du1 > 0.0f) {
                d = up2;
                e = (up0 - up2) * du2;
                f = (up1 - up2) * du2;
                y0 = du2 - du0;
                y1 = du2 - du1;
            } else if (du0du2 > 0.0f) {
                d = up1;
                e = (up0 - up1) * du1;
                f = (up2 - up1) * du1;
                y0 = du1 - du0;
                y1 = du1 - du2;
            } else if (du1 * du2 > 0.0f || du0 != 0.0f) {
                d = up0;
                e = (up1 - up0) * du0;
                f = (up2 - up0) * du0;
                y0 = du0 - du1;
                y1 = du0 - du2;
            } else if (du1 != 0.0f) {
                d = up1;
                e = (up0 - up1) * du1;
                f = (up2 - up1) * du1;
                y0 = du1 - du0;
                y1 = du1 - du2;
            } else if (du2 != 0.0f) {
                d = up2;
                e = (up0 - up2) * du2;
                f = (up1 - up2) * du2;
                y0 = du2 - du0;
                y1 = du2 - du1;
            } else { return coplanar_tri_tri(N1, V0, V1, V2, U0, U1, U2); }
        };

        double xx, yy, xxyy, tmp;
        xx = x0 * x1;
        yy = y0 * y1;
        xxyy = xx * yy;

        tmp = a * xxyy;
        isect1[0] = tmp + b * x1 * yy;
        isect1[1] = tmp + c * x0 * yy;

        tmp = d * xxyy;
        isect2[0] = tmp + e * xx * y1;
        isect2[1] = tmp + f * xx * y0;

        if (isect1[0] > isect1[1]) {
            double c;
            c = isect1[0];
            isect1[0] = isect1[1];
            isect1[1] = c;
        };
        if (isect2[0] > isect2[1]) {
            double c;
            c = isect2[0];
            isect2[0] = isect2[1];
            isect2[1] = c;
        };

        if (isect1[1] < isect2[0] || isect2[1] < isect1[0]) return 0;
        return 1;
    }
}