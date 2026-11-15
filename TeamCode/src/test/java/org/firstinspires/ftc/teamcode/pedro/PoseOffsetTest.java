package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

public class PoseOffsetTest {
    private static final double EPS = 1e-9;

    @Test
    public void aligningMapsRawOntoTarget() {
        PoseOffset o = PoseOffset.aligning(3, -4, 0.5, 72, 24, Math.PI / 2);
        assertArrayEquals(new double[]{72, 24, Math.PI / 2}, o.apply(3, -4, 0.5), EPS);
    }

    @Test
    public void rawMotionIsRotatedIntoTheField() {
        // raw heading 0, field heading 90: raw +x is field +y
        PoseOffset o = PoseOffset.aligning(0, 0, 0, 10, 10, Math.PI / 2);
        assertArrayEquals(new double[]{10, 15, Math.PI / 2}, o.apply(5, 0, 0), EPS);
        assertArrayEquals(new double[]{0, 2}, o.rotate(2, 0), EPS);
    }

    @Test
    public void wrapStaysInHalfOpenRange() {
        assertEquals(Math.PI, PoseOffset.wrap(-Math.PI), EPS);
        assertEquals(-Math.PI / 2, PoseOffset.wrap(3 * Math.PI / 2), EPS);
        assertEquals(0.25, PoseOffset.wrap(0.25 + 4 * Math.PI), EPS);
    }
}
