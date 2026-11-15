package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class PIDFControllerExTest {
    private static final double EPS = 1e-9;

    @Test
    public void svaFeedforwardAtASteadySetpoint() {
        PIDFControllerEx c = new PIDFControllerEx(0, 0, 0).setSVA(0.05, 0.0002, 0.001);
        c.calculate(0, 1000, 0.01);
        assertEquals(0.05 + 0.2, c.calculate(0, 1000, 0.01), EPS);
    }

    @Test
    public void kaUsesSetpointAcceleration() {
        PIDFControllerEx c = new PIDFControllerEx(0, 0, 0).setSVA(0, 0, 0.001);
        c.calculate(0, 0, 0.01);
        assertEquals(0.001 * 1 / 0.01, c.calculate(0, 1, 0.01), EPS);
    }

    @Test
    public void ksNeedsANonzeroSetpoint() {
        PIDFControllerEx c = new PIDFControllerEx(0, 0, 0).setSVA(0.1, 0, 0);
        assertEquals(0, c.calculate(0, 0, 0.01), EPS);
        assertEquals(-0.1, c.calculate(0, -5, 0.01), EPS);
    }

    @Test
    public void outputIsClamped() {
        PIDFControllerEx c = new PIDFControllerEx(1, 0, 0);
        assertEquals(1, c.calculate(0, 100, 0.01), EPS);
        assertEquals(-1, c.calculate(0, -100, 0.01), EPS);
    }

    @Test
    public void proportionalOnError() {
        PIDFControllerEx c = new PIDFControllerEx(0.01, 0, 0);
        assertEquals(0.5, c.calculate(50, 100, 0.01), EPS);
    }
}
