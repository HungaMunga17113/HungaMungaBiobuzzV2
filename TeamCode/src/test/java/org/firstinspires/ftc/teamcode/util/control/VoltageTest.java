package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class VoltageTest {
    private static final double EPS = 1e-9;

    @Test
    public void nominalVoltageIsUnchanged() {
        assertEquals(0.5, Voltage.compensate(0.5, Voltage.NOMINAL), EPS);
    }

    @Test
    public void lowBatteryGetsMorePower() {
        assertEquals(0.5 * 12.4 / 11.0, Voltage.compensate(0.5, 11.0), EPS);
    }

    @Test
    public void compensationIsClamped() {
        assertEquals(1, Voltage.compensate(0.95, 10), EPS);
        assertEquals(-1, Voltage.compensate(-0.95, 10), EPS);
    }

    @Test
    public void toNominalInvertsCompensate() {
        assertEquals(0.4, Voltage.toNominal(Voltage.compensate(0.4, 11.5), 11.5), EPS);
    }

    @Test
    public void garbageReadingsFallBackToNominal() {
        assertEquals(0.5, Voltage.compensate(0.5, 0), EPS);
        assertEquals(0.5, Voltage.compensate(0.5, Double.NaN), EPS);
        assertEquals(1, Voltage.maxNominal(0), EPS);
    }
}
