package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Hive;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.firstinspires.ftc.teamcode.util.control.PIDControllerEx;

import java.util.function.Supplier;

/**
 * axon (cr mode) + its analog encoder, pid + kS on turret degrees.
 * first encoder read sets the shaft count, so start facing forward when geared.
 * no encoder = unpowered.
 */
public final class Turret implements Subsystem {
    private final CRServo servo;
    private final AnalogInput encoder;
    private final PIDControllerEx pid = new PIDControllerEx(0, 0, 0);

    private double commandedRad, omega, measuredRad = Double.NaN, volts = Double.NaN;
    private double shaftRad, prevShaftRaw = Double.NaN;
    private double power, sentPower = Double.NaN, openPower = Double.NaN;
    private TurretAim.Solution last;

    public Turret(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;
    }

    /** robot frame, 0 = forward, ccw+, clamped to the ROM. stops auto-aim */
    public void setAngle(double rad) {
        last = null;
        omega = 0;
        commandedRad = TurretAim.fromConfig(Alliance.RED).clamp(rad);
    }

    /** points at the hive's up cell from the robot's pose, call every loop to keep tracking */
    public TurretAim.Solution aimAt(Pose pose, Velocity fieldVelocity, Hive hive) {
        last = TurretAim.fromConfig(hive.alliance()).solve(pose.x(), pose.y(), pose.heading(),
                fieldVelocity.vx, fieldVelocity.vy, hive.targetX(), hive.targetY(), commandedRad);
        omega = fieldVelocity.omega;
        commandedRad = last.turretRad;
        return last;
    }

    @Override
    public void periodic() {
        if (encoder != null) {
            volts = encoder.getVoltage();
            measuredRad = read(volts, encoder.getMaxVoltage());
        }
        double deg = Math.toDegrees(measuredRad);
        if (!Double.isNaN(openPower)) power = limit(deg, openPower);
        else power = encoder == null ? 0 : control(deg, Math.toDegrees(commandedRad));
        if (power != sentPower) {
            servo.setPower(power);
            sentPower = power;
        }
    }

    /** pid + kS toward the error, never pushing further past a limit */
    private double control(double deg, double targetDeg) {
        pid.setPID(TurretConfig.kP, TurretConfig.kI, TurretConfig.kD);
        double err = targetDeg - deg;
        double out = pid.calculate(deg, targetDeg) - TurretConfig.kW * Math.toDegrees(omega);
        if (Math.abs(err) > TurretConfig.TOLERANCE_DEG) out += TurretConfig.kS * Math.signum(err);
        return limit(deg, Range.clip(out, -1, 1));
    }

    private static double limit(double deg, double out) {
        return deg >= TurretConfig.MAX_DEG && out > 0 || deg <= TurretConfig.MIN_DEG && out < 0 ? 0 : out;
    }

    /** volts -> turret rad, counting shaft wraps between loops */
    private double read(double v, double maxV) {
        double raw = servoRad(v, maxV);
        shaftRad = Double.isNaN(prevShaftRaw) ? raw : shaftRad + TurretAim.wrap(raw - prevShaftRaw);
        prevShaftRaw = raw;
        return shaftRad * TurretConfig.ENCODER_RATIO;
    }

    /** one shaft rev over 0..maxV, zero at ENCODER_ZERO_V, (-pi, pi] */
    public static double servoRad(double v, double maxV) {
        return TurretAim.wrap((v - TurretConfig.ENCODER_ZERO_V) / maxV * 2 * Math.PI);
    }

    /** raw power for tuning, soft limits still apply. NaN = back to pid */
    public void setOpenLoop(double power) {
        if (Double.isNaN(power) && !Double.isNaN(openPower)) pid.reset();
        openPower = power;
    }

    public double angle() {
        return commandedRad;
    }

    /** NaN without an encoder */
    public double measuredAngle() {
        return measuredRad;
    }

    public double power() {
        return power;
    }

    public boolean hasEncoder() {
        return encoder != null;
    }

    /** within TOLERANCE_DEG of the command, true without an encoder */
    public boolean settled() {
        return encoder == null || Math.abs(Math.toDegrees(measuredRad - commandedRad)) <= TurretConfig.TOLERANCE_DEG;
    }

    /** tracking, target inside the rom, and the turret actually there */
    public boolean onTarget() {
        return last != null && last.reachable && settled();
    }

    /** latest auto-aim result, null when not tracking */
    public TurretAim.Solution solution() {
        return last;
    }

    // ivy commands, for auto

    /** keeps pointing at the hive's up cell from the robot's pose, until interrupted */
    public Command track(Supplier<Pose> pose, Supplier<Velocity> fieldVelocity, Hive hive) {
        return Command.build()
                .setExecute(() -> aimAt(pose.get(), fieldVelocity.get(), hive))
                .setEnd(e -> {
                    last = null;
                    omega = 0;
                })
                .requiring(this);
    }

    public Command waitUntilOnTarget() {
        return Commands.waitUntil(this::onTarget);
    }

    /** requires the turret, so it ends a running track() */
    public Command center() {
        return Commands.instant(() -> setAngle(0)).requiring(this);
    }

    @Override
    public void telemetry(Telemetry t) {
        if (encoder == null) {
            t.addData("turret", "%.0f°  NO ENCODER, unpowered", Math.toDegrees(commandedRad));
        } else {
            t.addData("turret", "%.1f° / %.1f°  pwr %.2f  (%.3f V)%s", Math.toDegrees(measuredRad), Math.toDegrees(commandedRad),
                    power, volts, settled() ? "" : "  moving");
        }
        if (last != null) {
            t.addData("aim", "%.0f in  %.0f°%s", last.distance, last.angleDeg,
                    last.reachable ? "" : String.format("  OUT OF RANGE, turn robot %+.0f°", Math.toDegrees(last.overshootRad)));
        }
    }
}
