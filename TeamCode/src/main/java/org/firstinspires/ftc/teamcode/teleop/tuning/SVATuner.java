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

import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.control.Voltage;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.util.tuning.SvaFit;
import org.firstinspires.ftc.teamcode.util.tuning.TuningFiles;

import java.io.File;
import java.util.List;
import java.util.Locale;

/**
 * open-loop ramp, coast, step, then fits kS kV kA at nominal volts. plots at URL.
 * b  abort
 */
@Config("SVATuner")
@TeleOp(name = "SVATuner", group = "Tuning")
public class SVATuner extends LinearOpMode {
    public static double RAMP_RATE = 0.08; // power/s
    public static double RAMP_MAX = 0.95;
    public static double STEP_POWER = 0.7;
    public static double STEP_S = 3;
    public static double COAST_MAX_S = 8;
    public static double MIN_RPM = 150;
    public static double ACCEL_WINDOW_S = 0.06;
    public static String URL = "http://192.168.43.1:8080/tuning/sva";

    private MotorGroup flywheel;
    private VoltageSensor battery;
    private List<LynxModule> hubs;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule h : hubs) h.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        flywheel = MotorGroup.of(hardwareMap, Names.FLYWHEEL, ConfigUtil.flywheelDirections());
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        battery = hardwareMap.voltageSensor.iterator().next();

        telemetry.addLine("flywheel ramps to " + RAMP_MAX + " power, then steps to " + STEP_POWER);
        telemetry.addLine("keep hands and balls clear, b aborts");
        telemetry.update();
        waitForStart();

        SvaFit.Samples ramp = new SvaFit.Samples(), step = new SvaFit.Samples();
        try {
            if (!record("ramp", ramp, t -> Math.min(RAMP_RATE * t, RAMP_MAX), RAMP_MAX / RAMP_RATE)) return;
            if (!coast()) return;
            if (!record("step", step, t -> STEP_POWER, STEP_S)) return;
        } finally {
            flywheel.setPower(0);
        }

        String status;
        SvaFit.Result r = null;
        try {
            r = SvaFit.fit(ramp, step, MIN_RPM, ACCEL_WINDOW_S);
            ShooterConfig.kS = r.kS;
            ShooterConfig.kV = r.kV;
            ShooterConfig.kA = r.kA;
            File f = TuningFiles.save(json(ramp, step, r));
            status = "saved " + f.getName();
        } catch (Exception e) {
            status = "!! " + e.getMessage();
        }

        while (opModeIsActive()) {
            telemetry.addLine(status);
            if (r != null) {
                telemetry.addData("kS", "%.5f", r.kS);
                telemetry.addData("kV", "%.4e", r.kV);
                telemetry.addData("kA", "%.4e", r.kA);
                telemetry.addData("r²", "%.4f  (%d samples)", r.r2, r.used);
                telemetry.addData("sdk f (approx)", "%.2f", SvaFit.sdkF(r.kV, ShooterConfig.TICKS_PER_REV, ShooterConfig.GEAR_RATIO));
                telemetry.addLine("copied into ShooterConfig until the rc restarts, paste them into RobotConfig");
            }
            telemetry.addLine("plots: " + URL);
            telemetry.update();
            sleep(100);
        }
    }

    private interface PowerFn {
        double at(double t);
    }

    private boolean record(String phase, SvaFit.Samples out, PowerFn power, double durationS) {
        long start = System.nanoTime();
        while (opModeIsActive()) {
            if (gamepad1.b) return false;
            double t = (System.nanoTime() - start) / 1e9;
            if (t > durationS) return true;
            for (LynxModule h : hubs) h.clearBulkCache();
            double p = power.at(t);
            flywheel.setPower(p);
            double volts = battery.getVoltage();
            double rpm = ConfigUtil.flywheelRpm(flywheel.getVelocity());
            out.add(t, p, volts, rpm);
            show(phase, t, p, volts, rpm, out.size());
        }
        return false;
    }

    private boolean coast() {
        long start = System.nanoTime();
        flywheel.setPower(0);
        while (opModeIsActive()) {
            if (gamepad1.b) return false;
            double t = (System.nanoTime() - start) / 1e9;
            for (LynxModule h : hubs) h.clearBulkCache();
            double rpm = ConfigUtil.flywheelRpm(flywheel.getVelocity());
            if (Math.abs(rpm) < MIN_RPM / 3 || t > COAST_MAX_S) return true;
            show("coast", t, 0, battery.getVoltage(), rpm, 0);
        }
        return false;
    }

    private void show(String phase, double t, double p, double volts, double rpm, int n) {
        telemetry.addData("phase", "%s  %.1fs", phase, t);
        telemetry.addData("power", "%.2f  (%.2f nominal)", p, Voltage.toNominal(p, volts));
        telemetry.addData("rpm", "%.0f  (spread %.0f)", rpm, ConfigUtil.flywheelRpm(flywheel.velocitySpread()));
        telemetry.addData("battery", "%.2f V", volts);
        telemetry.addData("samples", n);
        telemetry.update();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("power", p);
        packet.put("rpm", rpm);
        packet.put("battery (V)", volts);
        dashboard.sendTelemetryPacket(packet);
    }

    private static String json(SvaFit.Samples ramp, SvaFit.Samples step, SvaFit.Result r) {
        StringBuilder sb = new StringBuilder("{");
        sb.append("\"nominal\":").append(Voltage.NOMINAL)
                .append(",\"ticksPerRev\":").append(ShooterConfig.TICKS_PER_REV)
                .append(",\"gearRatio\":").append(ShooterConfig.GEAR_RATIO)
                .append(",\"minRpm\":").append(MIN_RPM)
                .append(",\"results\":{")
                .append(String.format(Locale.US, "\"kS\":%.8g,\"kV\":%.8g,\"kA\":%.8g,\"r2\":%.6f,\"used\":%d,\"sdkF\":%.6g",
                        r.kS, r.kV, r.kA, r.r2, r.used, SvaFit.sdkF(r.kV, ShooterConfig.TICKS_PER_REV, ShooterConfig.GEAR_RATIO)))
                .append("},\"ramp\":");
        samples(sb, ramp);
        sb.append(",\"step\":");
        samples(sb, step);
        return sb.append('}').toString();
    }

    private static void samples(StringBuilder sb, SvaFit.Samples s) {
        int stride = Math.max(1, s.size() / 1500);
        String[] keys = {"t", "p", "volts", "rpm"};
        List<?>[] cols = {s.t, s.power, s.volts, s.vel};
        sb.append('{');
        for (int k = 0; k < keys.length; k++) {
            sb.append(k == 0 ? "" : ",").append('"').append(keys[k]).append("\":[");
            for (int i = 0; i < s.size(); i += stride) {
                sb.append(i == 0 ? "" : ",").append(String.format(Locale.US, "%.5g", (Double) cols[k].get(i)));
            }
            sb.append(']');
        }
        sb.append('}');
    }
}
