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
import org.firstinspires.ftc.teamcode.util.control.Toggle;
import org.firstinspires.ftc.teamcode.util.control.Voltage;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;

import java.util.List;

/**
 * gains live in ShooterConfig (dashboard "Shooter"), MODE for controller picker
 * a           on / off
 * y           switch sdk pidf / ex
 * x           step test between STEP_LOW_RPM and targetRpm
 * dpad up/dn  targetRpm +/- RPM_STEP
 */
@Config("ShooterTuner")
@TeleOp(name = "ShooterTuner", group = "Tuning")
public class ShooterTunerOpMode extends LinearOpMode {
    public enum Mode { SDK_PIDF, EX }

    public static Mode MODE = Mode.EX;
    public static double RPM_STEP = 250;
    public static double STEP_LOW_RPM = 3000;
    public static double STEP_S = 2;
    public static boolean ALLOW_REVERSE = false;

    private MotorGroup flywheel;
    private DcMotor.RunMode runMode;
    private final double[] sentPidf = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        flywheel = MotorGroup.of(hardwareMap, Names.FLYWHEEL, ConfigUtil.flywheelDirections());
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        PIDFControllerEx ex = new PIDFControllerEx(0, 0, 0);
        StepStats stats = new StepStats();
        Toggle onButton = new Toggle(), modeButton = new Toggle(), stepButton = new Toggle(), up = new Toggle(), down = new Toggle();
        ElapsedTime stepTimer = new ElapsedTime(), voltTimer = new ElapsedTime();
        boolean on = false, stepping = false;
        Mode lastMode = MODE;
        double lastSetpoint = Double.NaN, volts = battery.getVoltage();

        telemetry.addLine("a starts the flywheel, keep clear");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            if (onButton.momentary(gamepad1.a)) on = !on;
            if (modeButton.momentary(gamepad1.y)) MODE = MODE == Mode.EX ? Mode.SDK_PIDF : Mode.EX;
            if (stepButton.momentary(gamepad1.x)) {
                stepping = !stepping;
                stepTimer.reset();
            }
            if (up.momentary(gamepad1.dpad_up)) ShooterConfig.targetRpm += RPM_STEP;
            if (down.momentary(gamepad1.dpad_down)) ShooterConfig.targetRpm = Math.max(0, ShooterConfig.targetRpm - RPM_STEP);
            if (MODE != lastMode) {
                ex.reset();
                lastMode = MODE;
            }

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
                setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                flywheel.setPower(0);
                ex.reset();
                power = 0;
            } else if (MODE == Mode.SDK_PIDF) {
                setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
                sendPidf(comp ? ShooterConfig.sdkF * Voltage.NOMINAL / volts : ShooterConfig.sdkF);
                flywheel.setVelocity(ConfigUtil.motorTps(setpoint));
                power = flywheel.getPower();
            } else {
                setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ex.setPID(ShooterConfig.kP, ShooterConfig.kI, ShooterConfig.kD);
                ex.setF(ShooterConfig.kF);
                ex.setSVA(ShooterConfig.kS, ShooterConfig.kV, ShooterConfig.kA);
                double max = comp ? Voltage.maxNominal(volts) : 1;
                ex.setOutputRange(ALLOW_REVERSE ? -max : 0, max);
                double out = ex.calculate(rpm, setpoint);
                power = comp ? Voltage.compensate(out, volts) : out;
                flywheel.setPower(power);
            }

            telemetry.addData("mode", "%s  %s%s", MODE, on ? "ON" : "off", stepping ? "  (step test)" : "");
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

    private void setRunMode(DcMotor.RunMode mode) {
        if (mode == runMode) return;
        flywheel.setMode(mode);
        runMode = mode;
    }

    /** only resend when something changed, f by more than 1% */
    private void sendPidf(double f) {
        double p = ShooterConfig.sdkP, i = ShooterConfig.sdkI, d = ShooterConfig.sdkD;
        boolean same = p == sentPidf[0] && i == sentPidf[1] && d == sentPidf[2]
                && Math.abs(f - sentPidf[3]) <= 0.01 * Math.abs(sentPidf[3]);
        if (same) return;
        flywheel.setVelocityPIDFCoefficients(p, i, d, f);
        sentPidf[0] = p;
        sentPidf[1] = i;
        sentPidf[2] = d;
        sentPidf[3] = f;
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
