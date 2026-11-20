package org.firstinspires.ftc.teamcode.util.tuning;

import org.firstinspires.ftc.teamcode.util.control.Voltage;

import java.util.ArrayList;
import java.util.List;

/** least squares fit of nominal power = kS*sgn(v) + kV*v + kA*a */
public final class SvaFit {
    private SvaFit() {}

    public static final class Samples {
        public final List<Double> t = new ArrayList<>(), power = new ArrayList<>(), volts = new ArrayList<>(), vel = new ArrayList<>();

        public void add(double time, double p, double v, double velocity) {
            t.add(time);
            power.add(p);
            volts.add(v);
            vel.add(velocity);
        }

        public int size() {
            return t.size();
        }

        double[] nominalPower() {
            double[] out = new double[size()];
            for (int i = 0; i < out.length; i++) out[i] = Voltage.toNominal(power.get(i), volts.get(i));
            return out;
        }
    }

    public static final class Result {
        public final double kS, kV, kA, r2;
        public final int used;

        Result(double kS, double kV, double kA, double r2, int used) {
            this.kS = kS;
            this.kV = kV;
            this.kA = kA;
            this.r2 = r2;
            this.used = used;
        }
    }

    /** ramp gives kS/kV, step gives kA; samples under minVel are dropped */
    public static Result fit(Samples ramp, Samples step, double minVel, double accelWindowSec) {
        List<double[]> rows = new ArrayList<>();
        collect(ramp, minVel, accelWindowSec, rows);
        collect(step, minVel, accelWindowSec, rows);
        if (rows.size() < 3) throw new IllegalArgumentException("not enough samples above " + minVel);

        double[][] ata = new double[3][3];
        double[] atb = new double[3];
        for (double[] r : rows) {
            double[] x = {Math.signum(r[0]), r[0], r[1]};
            for (int i = 0; i < 3; i++) {
                atb[i] += x[i] * r[2];
                for (int j = 0; j < 3; j++) ata[i][j] += x[i] * x[j];
            }
        }
        double[] k = solve3(ata, atb);

        double mean = 0;
        for (double[] r : rows) mean += r[2];
        mean /= rows.size();
        double ssRes = 0, ssTot = 0;
        for (double[] r : rows) {
            double pred = k[0] * Math.signum(r[0]) + k[1] * r[0] + k[2] * r[1];
            ssRes += (r[2] - pred) * (r[2] - pred);
            ssTot += (r[2] - mean) * (r[2] - mean);
        }
        return new Result(k[0], k[1], k[2], ssTot == 0 ? 1 : 1 - ssRes / ssTot, rows.size());
    }

    private static void collect(Samples s, double minVel, double window, List<double[]> rows) {
        double[] t = toArray(s.t), v = toArray(s.vel), p = s.nominalPower(), a = accel(t, v, window);
        for (int i = 0; i < t.length; i++) {
            if (Math.abs(v[i]) >= minVel) rows.add(new double[]{v[i], a[i], p[i]});
        }
    }

    /** local linear slope over +/- window/2 */
    public static double[] accel(double[] t, double[] v, double window) {
        double[] out = new double[t.length];
        int lo = 0, hi = 0;
        for (int i = 0; i < t.length; i++) {
            while (t[lo] < t[i] - window / 2) lo++;
            while (hi + 1 < t.length && t[hi + 1] <= t[i] + window / 2) hi++;
            double n = 0, st = 0, sv = 0, stt = 0, stv = 0;
            for (int j = lo; j <= hi; j++) {
                n++;
                st += t[j];
                sv += v[j];
                stt += t[j] * t[j];
                stv += t[j] * v[j];
            }
            double den = n * stt - st * st;
            out[i] = den == 0 ? 0 : (n * stv - st * sv) / den;
        }
        return out;
    }

    private static double[] toArray(List<Double> l) {
        double[] out = new double[l.size()];
        for (int i = 0; i < out.length; i++) out[i] = l.get(i);
        return out;
    }

    static double[] solve3(double[][] m, double[] b) {
        double[][] a = new double[3][4];
        for (int i = 0; i < 3; i++) {
            System.arraycopy(m[i], 0, a[i], 0, 3);
            a[i][3] = b[i];
        }
        for (int c = 0; c < 3; c++) {
            int piv = c;
            for (int r = c + 1; r < 3; r++) if (Math.abs(a[r][c]) > Math.abs(a[piv][c])) piv = r;
            double[] tmp = a[c];
            a[c] = a[piv];
            a[piv] = tmp;
            if (Math.abs(a[c][c]) < 1e-300) throw new IllegalArgumentException("singular fit, need both ramp and step data");
            for (int r = 0; r < 3; r++) {
                if (r == c) continue;
                double f = a[r][c] / a[c][c];
                for (int k = c; k < 4; k++) a[r][k] -= f * a[c][k];
            }
        }
        return new double[]{a[0][3] / a[0][0], a[1][3] / a[1][1], a[2][3] / a[2][2]};
    }
}
