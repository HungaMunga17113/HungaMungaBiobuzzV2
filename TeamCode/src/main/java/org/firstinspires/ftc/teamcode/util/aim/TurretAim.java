package org.firstinspires.ftc.teamcode.util.aim;

import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;

/**
 * turret angle to hit a field point.
 * field: inches, heading ccw+ rad. turret: robot frame, 0 = forward, ccw+, limited to [min, max].
 * clamp targets, overshootRad = chassis turn to restore rangw
 */
public final class TurretAim {
    public final double minRad, maxRad, offsetForward, offsetLeft, leadS;

    public TurretAim(double minDeg, double maxDeg, double offsetForward, double offsetLeft, double leadS) {
        if (minDeg >= maxDeg) throw new IllegalArgumentException("turret min " + minDeg + " >= max " + maxDeg);
        this.minRad = Math.toRadians(minDeg);
        this.maxRad = Math.toRadians(maxDeg);
        this.offsetForward = offsetForward;
        this.offsetLeft = offsetLeft;
        this.leadS = leadS;
    }

    /** from dash values, call again to pick up edits */
    public static TurretAim fromConfig() {
        return new TurretAim(TurretConfig.MIN_DEG, TurretConfig.MAX_DEG,
                TurretConfig.OFFSET_FORWARD, TurretConfig.OFFSET_LEFT, TurretConfig.LEAD_S);
    }

    public static final class Solution {
        /** clamped */
        public final double turretRad;
        /** ideal */
        public final double wantedRad;
        /** error if impossible */
        public final double overshootRad;
        /** turret pivot to the (lead-shifted) target, in; feed 2 rpm table */
        public final double distance;
        public final boolean reachable;

        Solution(double turretRad, double wantedRad, double distance) {
            this.turretRad = turretRad;
            this.wantedRad = wantedRad;
            this.overshootRad = wrap(wantedRad - turretRad);
            this.distance = distance;
            this.reachable = Math.abs(overshootRad) < 1e-9;
        }
    }

    /** robot pose + field vel (in/s, pass 0s for no lead) -> turret angle for target (tx, ty) */
    public Solution solve(double x, double y, double heading, double vx, double vy, double tx, double ty) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        double px = x + offsetForward * cos - offsetLeft * sin;
        double py = y + offsetForward * sin + offsetLeft * cos;

        double ax = tx - vx * leadS, ay = ty - vy * leadS;
        double dx = ax - px, dy = ay - py;

        double wanted = wrap(Math.atan2(dy, dx) - heading);
        return new Solution(clamp(wanted), wanted, Math.hypot(dx, dy));
    }

    /** closest around circle */
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
