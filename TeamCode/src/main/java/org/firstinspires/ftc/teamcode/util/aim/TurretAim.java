package org.firstinspires.ftc.teamcode.util.aim;

import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;
import org.firstinspires.ftc.teamcode.game.Alliance;

import java.util.function.DoubleBinaryOperator;

/**
 * turret angle to hit a field point
 * turret: robot frame, 0 = forward, ccw+, limited to [min, max].
 * clamp targets, overshootRad = chassis turn to restore range
 */
public final class TurretAim {
    public final double minRad, maxRad, offsetForward, offsetLeft;
    /** field heading subtracted from the goal->robot bearing, so mirrored alliances share shot angles */
    public final double refRad;
    /** (distance in, angle deg) -> ball flight s */
    public final DoubleBinaryOperator tof;

    /** constant flight time, ref 0 */
    public TurretAim(double minDeg, double maxDeg, double offsetForward, double offsetLeft, double leadS) {
        this(minDeg, maxDeg, offsetForward, offsetLeft, 0, (d, a) -> leadS);
    }

    public TurretAim(double minDeg, double maxDeg, double offsetForward, double offsetLeft, double refRad, DoubleBinaryOperator tof) {
        if (minDeg >= maxDeg) throw new IllegalArgumentException("turret min " + minDeg + " >= max " + maxDeg);
        this.minRad = Math.toRadians(minDeg);
        this.maxRad = Math.toRadians(maxDeg);
        this.offsetForward = offsetForward;
        this.offsetLeft = offsetLeft;
        this.refRad = refRad;
        this.tof = tof;
    }

    public static TurretAim fromConfig() {
        return new TurretAim(TurretConfig.MIN_DEG, TurretConfig.MAX_DEG,
                TurretConfig.OFFSET_FORWARD, TurretConfig.OFFSET_LEFT, Alliance.selected.driverForward, ShotTable.fromConfig()::tof);
    }

    public static final class Solution {
        public final double turretRad; // clamped
        public final double wantedRad; // ideal
        public final double overshootRad; // error if impossible
        public final double distance; // pivot to the target, (in), feed 2 rpm table
        public final double angleDeg; // goal -> pivot bearing minus refRad, the shot table's angle
        public final double tof; // ball lead time of flight
        public final boolean reachable;

        Solution(double turretRad, double wantedRad, double distance, double angleDeg, double tof) {
            this.turretRad = turretRad;
            this.wantedRad = wantedRad;
            this.overshootRad = wrap(wantedRad - turretRad);
            this.distance = distance;
            this.angleDeg = angleDeg;
            this.tof = tof;
            this.reachable = Math.abs(overshootRad) < 1e-9;
        }
    }

    /** robot pose + field vel (in/s, pass 0s for no lead) -> turret angle for target (tx, ty) */
    public Solution solve(double x, double y, double heading, double vx, double vy, double tx, double ty) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double px = x + offsetForward * cos - offsetLeft * sin;
        double py = y + offsetForward * sin + offsetLeft * cos;

        // tof <- dist <- lead
        double dx = tx - px, dy = ty - py, t = 0;
        for (int i = 0; i < 3; i++) {
            t = tof.applyAsDouble(Math.hypot(dx, dy), shotAngleDeg(dx, dy));
            dx = tx - vx * t - px;
            dy = ty - vy * t - py;
        }

        double wanted = wrap(Math.atan2(dy, dx) - heading);
        return new Solution(clamp(wanted), wanted, Math.hypot(dx, dy), shotAngleDeg(dx, dy), t);
    }

    /** pivot->target (dx, dy) -> goal->pivot bearing relative to refRad, deg */
    private double shotAngleDeg(double dx, double dy) {
        return Math.toDegrees(wrap(Math.atan2(-dy, -dx) - refRad));
    }

    public double clamp(double rad) {
        rad = wrap(rad);
        if (rad >= minRad && rad <= maxRad) return rad;
        double pastMax = Math.abs(wrap(rad - maxRad)), pastMin = Math.abs(wrap(minRad - rad));
        return pastMax <= pastMin ? maxRad : minRad;
    }

    /** to (-pi, pi] */
    public static double wrap(double rad) {
        double r = rad % (2 * Math.PI);
        if (r <= -Math.PI) r += 2 * Math.PI;
        if (r > Math.PI) r -= 2 * Math.PI;
        return r;
    }
}
