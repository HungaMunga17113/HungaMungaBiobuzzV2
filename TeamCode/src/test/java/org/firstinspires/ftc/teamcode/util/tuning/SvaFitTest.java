package org.firstinspires.ftc.teamcode.util.tuning;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.util.control.Voltage;
import org.junit.Test;

import java.util.Random;

public class SvaFitTest {
    private static final double KS = 0.04, KV = 1.6e-4, KA = 6e-5;
    private static final double DT = 0.005;

    /** flywheel sim on a sagging battery, nominal-power model */
    private static double v;

    private static void run(SvaFit.Samples s, double seconds, java.util.function.DoubleUnaryOperator power, Random noise) {
        for (double t = 0; t < seconds; t += DT) {
            double p = power.applyAsDouble(t);
            double volts = 12.6 - 1.5 * Math.abs(p) + 0.02 * noise.nextGaussian();
            double pn = Voltage.toNominal(p, volts);
            double a = v > 1e-6 || pn > KS ? (pn - KS * Math.signum(Math.max(v, 1e-9)) - KV * v) / KA : 0;
            v = Math.max(0, v + a * DT);
            if (s != null) s.add(t, p, volts, v + 5 * noise.nextGaussian());
        }
    }

    @Test
    public void recoversGainsThroughVoltageSag() {
        Random noise = new Random(1);
        SvaFit.Samples ramp = new SvaFit.Samples(), step = new SvaFit.Samples();
        v = 0;
        run(ramp, 12, t -> Math.min(0.08 * t, 0.95), noise);
        run(null, 6, t -> 0, noise);
        run(step, 3, t -> 0.7, noise);

        SvaFit.Result r = SvaFit.fit(ramp, step, 100, 0.06);
        assertEquals(KS, r.kS, KS * 0.1);
        assertEquals(KV, r.kV, KV * 0.03);
        assertEquals(KA, r.kA, KA * 0.15);
        assertTrue(r.r2 > 0.99);
    }

    @Test
    public void accelOfALineIsItsSlope() {
        double[] t = new double[100], x = new double[100];
        for (int i = 0; i < 100; i++) {
            t[i] = i * 0.01;
            x[i] = 3 + 250 * t[i];
        }
        for (double a : SvaFit.accel(t, x, 0.05)) assertEquals(250, a, 1e-6);
    }

    @Test
    public void sdkFMatchesTheHubConvention() {
        // kV = 1 / max rpm -> f = 32767 / max tps
        double maxRpm = 6000, tpr = 28;
        assertEquals(32767 / (maxRpm * tpr / 60), SvaFit.sdkF(1 / maxRpm, tpr, 1), 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void refusesEmptyData() {
        SvaFit.fit(new SvaFit.Samples(), new SvaFit.Samples(), 100, 0.05);
    }
}
