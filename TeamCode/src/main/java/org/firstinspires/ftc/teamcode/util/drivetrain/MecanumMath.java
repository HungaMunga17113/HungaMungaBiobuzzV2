package org.firstinspires.ftc.teamcode.util.drivetrain;

/** +forward, +strafe right, +turn cw, heading ccw+ rad */
public final class MecanumMath {
    private MecanumMath() {}

    /** fl, bl, fr, br, scaled so none exceed 1 */
    public static double[] mix(double forward, double strafe, double turn) {
        double d = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        return new double[]{
                (forward + strafe + turn) / d,
                (forward - strafe + turn) / d,
                (forward - strafe - turn) / d,
                (forward + strafe - turn) / d
        };
    }

    /** driver-frame stick -> robot frame, returns {strafe, forward} */
    public static double[] fieldToRobot(double strafe, double forward, double heading) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        return new double[]{strafe * cos + forward * sin, forward * cos - strafe * sin};
    }

    /** zero inside db, rescaled so the output still reaches 1 */
    public static double deadband(double input, double db) {
        if (Math.abs(input) < db) return 0;
        return (input - Math.signum(input) * db) / (1 - db);
    }
}
