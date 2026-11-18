package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * todo: motor/servo + encoder, then setAngle() drives it (position pid, Conversions.degreesToTicks).
 */
public final class Turret implements Subsystem {
    private double commandedRad;
    private TurretAim.Solution last;

    /** robot frame, 0 = forward, ccw+, clamped to the ROM. stops auto-aim */
    public void setAngle(double rad) {
        last = null;
        apply(rad);
    }

    /** points at (tx, ty) from the robot's pose, call every loop to keep tracking */
    public TurretAim.Solution aimAt(Pose pose, Velocity fieldVelocity, double tx, double ty) {
        TurretAim.Solution s = TurretAim.fromConfig().solve(pose.x(), pose.y(), pose.heading(),
                fieldVelocity.vx, fieldVelocity.vy, tx, ty);
        apply(s.turretRad);
        last = s;
        return s;
    }

    private void apply(double rad) {
        commandedRad = TurretAim.fromConfig().clamp(rad);
        // todo: hardware
    }

    public double angle() {
        return commandedRad;
    }

    /** latest auto-aim result, null when not tracking */
    public TurretAim.Solution solution() {
        return last;
    }

    // ivy commands, for auto

    /** keeps pointing at (tx, ty) from the robot's pose, until interrupted */
    public Command track(Supplier<Pose> pose, Supplier<Velocity> fieldVelocity, DoubleSupplier tx, DoubleSupplier ty) {
        return Command.build()
                .setExecute(() -> aimAt(pose.get(), fieldVelocity.get(), tx.getAsDouble(), ty.getAsDouble()))
                .setEnd(e -> last = null)
                .requiring(this);
    }

    public Command center() {
        return Commands.instant(() -> setAngle(0)).requiring(this);
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("turret", "%.0f°", Math.toDegrees(commandedRad));
        if (last != null) {
            t.addData("aim", "%.0f in%s", last.distance,
                    last.reachable ? "" : String.format("  OUT OF RANGE, turn robot %+.0f°", Math.toDegrees(last.overshootRad)));
        }
    }
}
