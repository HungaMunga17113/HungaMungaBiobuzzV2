package org.firstinspires.ftc.teamcode.util.drivetrain;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MecanumMathTest {
    private static final double EPS = 1e-9;

    @Test
    public void forwardSpinsEveryWheelForward() {
        assertArrayEquals(new double[]{0.5, 0.5, 0.5, 0.5}, MecanumMath.mix(0.5, 0, 0), EPS);
    }

    @Test
    public void strafeRightIsTheXPattern() {
        assertArrayEquals(new double[]{0.5, -0.5, -0.5, 0.5}, MecanumMath.mix(0, 0.5, 0), EPS);
    }

    @Test
    public void turnClockwiseDrivesLeftSideForward() {
        assertArrayEquals(new double[]{0.5, 0.5, -0.5, -0.5}, MecanumMath.mix(0, 0, 0.5), EPS);
    }

    @Test
    public void mixNeverExceedsOneAndKeepsRatios() {
        double[] p = MecanumMath.mix(1, 1, 1);
        for (double v : p) assertTrue(Math.abs(v) <= 1 + EPS);
        assertEquals(1.0, p[0], EPS);
        assertEquals(p[1], -p[2], EPS);
    }

    @Test
    public void fieldCentricIsIdentityAtZeroHeading() {
        assertArrayEquals(new double[]{0.3, 0.7}, MecanumMath.fieldToRobot(0.3, 0.7, 0), EPS);
    }

    @Test
    public void robotFacingLeftStrafesRightForFieldForward() {
        // robot turned 90 ccw, driver pushes away -> robot has to strafe right
        assertArrayEquals(new double[]{1, 0}, MecanumMath.fieldToRobot(0, 1, Math.PI / 2), EPS);
    }

    @Test
    public void robotFacingRightStrafesLeftForFieldForward() {
        assertArrayEquals(new double[]{-1, 0}, MecanumMath.fieldToRobot(0, 1, -Math.PI / 2), EPS);
    }

    @Test
    public void deadbandZerosSmallInputAndStillReachesOne() {
        assertEquals(0, MecanumMath.deadband(0.02, 0.03), EPS);
        assertEquals(1, MecanumMath.deadband(1, 0.03), EPS);
        assertEquals(-1, MecanumMath.deadband(-1, 0.03), EPS);
        assertTrue(MecanumMath.deadband(0.04, 0.03) > 0);
    }
}
