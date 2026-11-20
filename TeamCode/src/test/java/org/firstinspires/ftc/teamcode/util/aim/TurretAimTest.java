package org.firstinspires.ftc.teamcode.util.aim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class TurretAimTest {
    private static final double EPS = 1e-9;
    private final TurretAim aim = new TurretAim(-170, 170, 0, 0, 0);

    private static double deg(double d) {
        return Math.toRadians(d);
    }

    @Test
    public void targetStraightAheadIsZero() {
        TurretAim.Solution s = aim.solve(0, 0, 0, 0, 0, 10, 0);
        assertEquals(0, s.turretRad, EPS);
        assertEquals(10, s.distance, EPS);
        assertTrue(s.reachable);
    }

    @Test
    public void leftIsPositive() {
        // facing +x, target on the robot's left (+y)
        assertEquals(deg(90), aim.solve(0, 0, 0, 0, 0, 0, 10).turretRad, EPS);
        // facing +y, target at +x is on the robot's right
        assertEquals(deg(-90), aim.solve(0, 0, deg(90), 0, 0, 10, 0).turretRad, EPS);
    }

    @Test
    public void wrapsAcrossPi() {
        // facing 170 deg, target at -170 deg field: 20 deg to the left, not 340 to the right
        TurretAim.Solution s = aim.solve(0, 0, deg(170), 0, 0, Math.cos(deg(-170)), Math.sin(deg(-170)));
        assertEquals(deg(20), s.turretRad, 1e-6);
    }

    @Test
    public void deadZoneClampsToNearerLimit() {
        // target 175 deg left of forward: past +170
        TurretAim.Solution s = aim.solve(0, 0, 0, 0, 0, Math.cos(deg(175)), Math.sin(deg(175)));
        assertFalse(s.reachable);
        assertEquals(deg(170), s.turretRad, 1e-6);
        assertEquals(deg(5), s.overshootRad, 1e-6);
        // 175 to the right
        s = aim.solve(0, 0, 0, 0, 0, Math.cos(deg(-175)), Math.sin(deg(-175)));
        assertEquals(deg(-170), s.turretRad, 1e-6);
        assertEquals(deg(-5), s.overshootRad, 1e-6);
    }

    @Test
    public void asymmetricRangeClampsAroundTheBack() {
        TurretAim a = new TurretAim(-90, 120, 0, 0, 0);
        // -150 is 60 past -90 but only 90 past 120 the other way around: -90 is nearer
        assertEquals(deg(-90), a.clamp(deg(-150)), 1e-6);
        // 170 is 50 past 120 and 100 past -90 the other way: 120
        assertEquals(deg(120), a.clamp(deg(170)), 1e-6);
    }

    @Test
    public void holdsItsLimitBehindTheRobot() {
        TurretAim a = new TurretAim(-150, 150, 0, 0, 0);
        assertEquals(deg(150), a.clamp(deg(-155), deg(150)), 1e-6);
        assertEquals(deg(-150), a.clamp(deg(155), deg(-150)), 1e-6);
        assertEquals(deg(-140), a.clamp(deg(-140), deg(150)), 1e-6);
        assertEquals(deg(-150), a.clamp(deg(-155), deg(100)), 1e-6);
    }

    @Test
    public void turretOffsetRotatesWithTheRobot() {
        // pivot 5 in behind center; facing +y puts the pivot at (0, -5)
        TurretAim a = new TurretAim(-170, 170, -5, 0, 0);
        TurretAim.Solution s = a.solve(0, 0, deg(90), 0, 0, 0, 10);
        assertEquals(15, s.distance, EPS);
        assertEquals(0, s.turretRad, EPS);
    }

    @Test
    public void leadAimsAgainstVelocity() {
        // driving +y at 20 in/s with 0.5 s flight: aim 10 in behind the target in y
        TurretAim a = new TurretAim(-170, 170, 0, 0, 0.5);
        TurretAim.Solution s = a.solve(0, 0, 0, 0, 20, 10, 10);
        assertEquals(0, s.turretRad, EPS);
        assertEquals(10, s.distance, EPS);
    }

    @Test
    public void leadUsesTofForTheLeadDistance() {
        // tof 0.01 s per in; driving +y at 20 in/s, target 50 in ahead: converges to ~0.5 s, ~10 in of lead
        TurretAim a = new TurretAim(-170, 170, 0, 0, 0, (d, ang) -> d / 100);
        TurretAim.Solution s = a.solve(0, 0, 0, 0, 20, 50, 0);
        assertEquals(s.distance / 100, s.tof, 1e-3);
        assertEquals(-20 * s.tof, Math.tan(s.turretRad) * 50, 1e-2);
    }

    @Test
    public void shotAngleIsGoalToRobotBearingMinusRef() {
        // robot straight below the goal: bearing goal->robot = -90
        assertEquals(-90, aim.solve(0, 0, 0, 0, 0, 0, 50).angleDeg, 1e-9);
        // red ref (180) rotates it to +90, same as blue mirrored above the goal
        TurretAim red = new TurretAim(-170, 170, 0, 0, Math.PI, (d, ang) -> 0);
        assertEquals(90, red.solve(0, 0, 0, 0, 0, 0, 50).angleDeg, 1e-9);
    }

    @Test
    public void wrap() {
        assertEquals(Math.PI, TurretAim.wrap(-Math.PI), EPS);
        assertEquals(deg(-90), TurretAim.wrap(deg(270)), 1e-9);
        assertEquals(deg(10), TurretAim.wrap(deg(370 + 720)), 1e-9);
    }
}
