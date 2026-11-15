package org.firstinspires.ftc.teamcode.pedro;

/** rigid transform raw -> field */
public final class PoseOffset {
    public static final PoseOffset IDENTITY = new PoseOffset(0, 0, 0);

    public final double dx, dy, dTheta;
    private final double cos, sin;

    public PoseOffset(double dx, double dy, double dTheta) {
        this.dx = dx;
        this.dy = dy;
        this.dTheta = dTheta;
        this.cos = Math.cos(dTheta);
        this.sin = Math.sin(dTheta);
    }

    /** offset that maps raw pose (rx, ry, rh) onto target (tx, ty, th) */
    public static PoseOffset aligning(double rx, double ry, double rh, double tx, double ty, double th) {
        double d = th - rh, c = Math.cos(d), s = Math.sin(d);
        return new PoseOffset(tx - (c * rx - s * ry), ty - (s * rx + c * ry), d);
    }

    public double[] apply(double x, double y, double heading) {
        return new double[]{cos * x - sin * y + dx, sin * x + cos * y + dy, wrap(heading + dTheta)};
    }

    public double[] rotate(double vx, double vy) {
        return new double[]{cos * vx - sin * vy, sin * vx + cos * vy};
    }

    public static double wrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad <= -Math.PI) rad += 2 * Math.PI;
        return rad;
    }
}
