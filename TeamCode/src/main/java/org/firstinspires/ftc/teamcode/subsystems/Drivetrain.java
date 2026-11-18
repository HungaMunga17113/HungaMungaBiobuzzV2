package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumMath;


public final class Drivetrain implements Subsystem {
    private final Follower follower;

    public Drivetrain(Follower follower) {
        this.follower = follower;
    }

    public Follower follower() {
        return follower;
    }

    public Pose pose() {
        return follower.pose();
    }

    /**
     * driver inputs: +right, +forward (away from the driver), +cw turn.
     * driverHeading = robot heading relative to "away from the driver" (rad ccw+), NaN for robot-centric.
     */
    public void drive(double right, double forward, double cw, double driverHeading) {
        double[] r = Double.isNaN(driverHeading)
                ? new double[]{right, forward}
                : MecanumMath.fieldToRobot(right, forward, driverHeading);
        // pedro: +forward, +strafe left, +turn ccw
        follower.manual(r[1], -r[0], -cw);
    }

    public Command follow(Path path) {
        return PedroCommands.follow(follower, path).requiring(this);
    }

    public Command hold(Pose pose) {
        return PedroCommands.hold(follower, pose).requiring(this);
    }

    public void relocalize(Pose pose) {
        follower.setPose(pose);
    }

    @Override
    public void telemetry(Telemetry t) {
        Pose p = pose();
        t.addData("pose", "(%.1f, %.1f) %.0f°", p.x(), p.y(), Math.toDegrees(p.heading()));
    }
}
