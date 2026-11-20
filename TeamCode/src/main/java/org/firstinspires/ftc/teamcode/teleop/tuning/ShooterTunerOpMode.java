package org.firstinspires.ftc.teamcode.teleop.tuning;

import org.firstinspires.ftc.teamcode.util.ConfigUtil;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.control.PIDFControllerEx;
import org.firstinspires.ftc.teamcode.util.control.Voltage;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;

import java.util.List;

/**
 * gains live in ShooterConfig (dashboard "Shooter")
 * a           on / off
 * x           step test between STEP_LOW_RPM and targetRpm
 * dpad up/dn  targetRpm +/- RPM_STEP
 */
@Config("ShooterTuner")
@TeleOp(name = "ShooterTuner", group = "Tuning")
public class ShooterTunerOpMode extends LinearOpMode {
    public static double RPM_STEP = 250;
    public static double STEP_LOW_RPM = 3000;
    public static double STEP_S = 2;
    public static boolean ALLOW_REVERSE = false;

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        MotorGroup flywheel = MotorGroup.of(hardwareMap, Names.FLYWHEEL, ConfigUtil.flywheelDirections());
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PIDFControllerEx ex = new PIDFControllerEx(0, 0, 0);
        StepStats stats = new StepStats();
        ElapsedTime stepTimer = new ElapsedTime(), voltTimer = new ElapsedTime();
        boolean on = false, stepping = false;
        double lastSetpoint = Double.NaN, volts = battery.getVoltage();

        telemetry.addLine("a starts the flywheel, keep clear");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.aWasPressed()) on = !on;
            if (gamepad1.xWasPressed()) {
                stepping = !stepping;
                stepTimer.reset();
            }
            if (gamepad1.dpadUpWasPressed()) ShooterConfig.targetRpm += RPM_STEP;
            if (gamepad1.dpadDownWasPressed()) ShooterConfig.targetRpm = Math.max(0, ShooterConfig.targetRpm - RPM_STEP);

            for (LynxModule h : hubs) h.clearBulkCache();
            if (voltTimer.milliseconds() > 100) {
                volts += 0.3 * (battery.getVoltage() - volts);
                voltTimer.reset();
            }
            double rpm = ConfigUtil.flywheelRpm(flywheel.getVelocity());
            double setpoint = !on ? 0
                    : stepping && ((int) (stepTimer.seconds() / STEP_S)) % 2 == 1 ? STEP_LOW_RPM
                    : ShooterConfig.targetRpm;
            if (setpoint != lastSetpoint) {
                stats.start(rpm, setpoint);
                lastSetpoint = setpoint;
            }
            stats.update(rpm);

            boolean comp = ShooterConfig.voltageComp;
            double power;
            if (!on) {
                flywheel.setPower(0);
                ex.reset();
                power = 0;
            } else {
                ex.setPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);
                ex.setSVA(ShooterConfig.kS, ShooterConfig.kV, ShooterConfig.kA);
                double max = comp ? Voltage.maxNominal(volts) : 1;
                ex.setOutputRange(ALLOW_REVERSE ? -max : 0, max);
                double out = ex.calculate(rpm, setpoint);
                power = comp ? Voltage.compensate(out, volts) : out;
                flywheel.setPower(power);
            }

            telemetry.addData("mode", "%s%s", on ? "ON" : "off", stepping ? "  (step test)" : "");
            telemetry.addData("target", "%.0f rpm", setpoint);
            telemetry.addData("actual", "%.0f rpm  (err %+.0f)", rpm, setpoint - rpm);
            telemetry.addData("power", "%.3f", power);
            telemetry.addData("motors", "%d  spread %.0f rpm  %.1f A", flywheel.size(),
                    ConfigUtil.flywheelRpm(flywheel.velocitySpread()), flywheel.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("battery", "%.2f V%s", volts, comp ? "  (compensating)" : "");
            telemetry.addData("rise 90%", Double.isNaN(stats.riseS) ? "-" : String.format("%.2f s", stats.riseS));
            telemetry.addData("overshoot", "%.1f %%", stats.overshootPct());
            telemetry.addData("avg |err|", "%.0f rpm", stats.avgAbsErr);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target rpm", setpoint);
            packet.put("rpm", rpm);
            packet.put("error rpm", setpoint - rpm);
            packet.put("power", power);
            packet.put("battery (V)", volts);
            dashboard.sendTelemetryPacket(packet);
        }
        flywheel.setPower(0);
    }

    private static final class StepStats {
        double startRpm, target, peak, riseS = Double.NaN, avgAbsErr;
        long startNanos;

        void start(double rpm, double setpoint) {
            startRpm = rpm;
            target = setpoint;
            peak = rpm;
            riseS = Double.NaN;
            startNanos = System.nanoTime();
        }

        void update(double rpm) {
            double dir = Math.signum(target - startRpm);
            if (dir * rpm > dir * peak) peak = rpm;
            if (Double.isNaN(riseS) && dir != 0 && dir * (rpm - startRpm) >= 0.9 * Math.abs(target - startRpm)) {
                riseS = (System.nanoTime() - startNanos) / 1e9;
            }
            avgAbsErr += 0.05 * (Math.abs(target - rpm) - avgAbsErr);
        }

        double overshootPct() {
            double span = Math.abs(target - startRpm);
            if (span < 1) return 0;
            return Math.max(0, Math.signum(target - startRpm) * (peak - target)) / span * 100;
        }
    }
}
