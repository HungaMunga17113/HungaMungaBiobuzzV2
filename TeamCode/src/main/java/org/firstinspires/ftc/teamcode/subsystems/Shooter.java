package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.util.ConfigUtil;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.aim.ShotTable;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.firstinspires.ftc.teamcode.util.control.PIDFControllerEx;
import org.firstinspires.ftc.teamcode.util.control.Voltage;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;

import java.util.function.DoubleSupplier;

/** todo: tune shooter */
public final class Shooter implements Subsystem {
    private final MotorGroup flywheel;
    private final DoubleSupplier volts;
    private final PIDFControllerEx controller = new PIDFControllerEx(0, 0, 0);
    private double targetRpm, rpm, power, sentPower = Double.NaN;

    public Shooter(MotorGroup flywheel, DoubleSupplier volts) {
        this.flywheel = flywheel;
        this.volts = volts;
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** ShooterConfig.SHOTS at the aim's distance + angle, targetRpm without an aim or table */
    public static double rpmFor(TurretAim.Solution aim) {
        double rpm = aim == null ? Double.NaN : ShotTable.fromConfig().rpm(aim.distance, aim.angleDeg);
        return Double.isNaN(rpm) ? ShooterConfig.targetRpm : rpm;
    }

    public void setTarget(double rpm) {
        if (rpm <= 0 && targetRpm > 0) controller.reset();
        targetRpm = Math.max(0, rpm);
    }

    public double target() {
        return targetRpm;
    }

    public boolean on() {
        return targetRpm > 0;
    }

    public boolean atSpeed() {
        return on() && Math.abs(targetRpm - rpm) <= ShooterConfig.READY_TOLERANCE_RPM;
    }

    public Command spinUp(double rpm) {
        return instant(() -> setTarget(rpm)).requiring(this);
    }

    public Command stop() {
        return instant(() -> setTarget(0)).requiring(this);
    }

    public Command waitUntilReady() {
        return waitUntil(this::atSpeed);
    }

    @Override
    public void periodic() {
        rpm = ConfigUtil.flywheelRpm(flywheel.getVelocity());
        if (!on()) {
            power = 0;
        } else {
            controller.setPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);
            controller.setF(ShooterConfig.kF);
            controller.setSVA(ShooterConfig.kS, ShooterConfig.kV, ShooterConfig.kA);
            double v = volts.getAsDouble();
            boolean comp = ShooterConfig.voltageComp && !Double.isNaN(v);
            controller.setOutputRange(0, comp ? Voltage.maxNominal(v) : 1);
            double out = controller.calculate(rpm, targetRpm);
            power = comp ? Voltage.compensate(out, v) : out;
        }
        if (power != sentPower) {
            flywheel.setPower(power);
            sentPower = power;
        }
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("shooter", "%s %.0f / %.0f rpm  %.2f pwr", atSpeed() ? "READY" : on() ? "spinning" : "off", rpm, targetRpm, power);
    }
}
