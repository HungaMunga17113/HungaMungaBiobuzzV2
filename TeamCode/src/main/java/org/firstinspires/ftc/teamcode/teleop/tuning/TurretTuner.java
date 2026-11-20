package org.firstinspires.ftc.teamcode.teleop.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

/**
 * gains live in TurretConfig (dashboard "Turret"), graph on the dashboard
 * a           step test STEP_A_DEG <-> STEP_B_DEG every STEP_S
 * dpad l/r    hold target +/- JOG_DEG
 * y           open loop: left stick x = raw power (find kS, check direction)
 * b           zero: hold 0
 *
 * order: ENCODER_ZERO_V + ENCODER_RATIO sign (open loop, push left = ccw = deg up),
 * kS (smallest power that moves it), kP until it overshoots, kD to damp, kI only if it sits short
 */
@Config("TurretTuner")
@TeleOp(name = "TurretTuner", group = "Tuning")
public class TurretTuner extends LinearOpMode {
    public static double STEP_A_DEG = -45;
    public static double STEP_B_DEG = 45;
    public static double STEP_S = 1.5;
    public static double JOG_DEG = 10;
    public static double OPEN_MAX = 0.3;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotHardware robot = new RobotHardware(hardwareMap);
        AnalogInput encoder = robot.optionalAnalog(Names.TURRET_ENCODER);
        Turret turret = new Turret(robot.optionalCrServo(Names.TURRET), encoder);

        robot.addInitWarnings(telemetry);
        if (encoder == null) telemetry.addLine("NO ENCODER, only open loop works");
        telemetry.addLine("start facing forward");
        telemetry.update();
        waitForStart();

        StepStats stats = new StepStats();
        ElapsedTime stepTimer = new ElapsedTime(), loop = new ElapsedTime();
        boolean stepping = false, open = false;
        double hold = 0, lastTarget = Double.NaN, lastDeg = Double.NaN;

        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.aWasPressed()) {
                stepping = !stepping;
                open = false;
                stepTimer.reset();
            }
            if (gamepad1.yWasPressed()) {
                open = !open;
                stepping = false;
            }
            if (gamepad1.bWasPressed()) {
                hold = 0;
                stepping = open = false;
            }
            if (gamepad1.dpadLeftWasPressed()) hold += JOG_DEG;
            if (gamepad1.dpadRightWasPressed()) hold -= JOG_DEG;

            double target = stepping
                    ? (((int) (stepTimer.seconds() / STEP_S)) % 2 == 0 ? STEP_A_DEG : STEP_B_DEG)
                    : hold;
            turret.setAngle(Math.toRadians(target));
            target = Math.toDegrees(turret.angle());
            turret.setOpenLoop(open ? -gamepad1.left_stick_x * OPEN_MAX : Double.NaN);
            turret.periodic();

            double deg = Math.toDegrees(turret.measuredAngle());
            double dt = loop.seconds();
            loop.reset();
            double vel = dt > 0 ? (deg - lastDeg) / dt : Double.NaN;
            lastDeg = deg;
            if (!open && target != lastTarget) stats.start(deg, target);
            lastTarget = open ? Double.NaN : target;
            if (!open) stats.update(deg);

            telemetry.addData("mode", open ? "OPEN LOOP (left stick x)" : stepping ? "step test" : "hold");
            telemetry.addData("target", "%.1f°", target);
            telemetry.addData("measured", "%.1f°  (err %+.1f)", deg, target - deg);
            telemetry.addData("velocity", "%.0f °/s", vel);
            telemetry.addData("power", "%.3f", turret.power());
            telemetry.addData("encoder", encoder == null ? "missing" : String.format("%.3f / %.2f V", encoder.getVoltage(), encoder.getMaxVoltage()));
            telemetry.addData("gains", "p %.4f  i %.4f  d %.4f  s %.3f", TurretConfig.kP, TurretConfig.kI, TurretConfig.kD, TurretConfig.kS);
            if (!open) {
                telemetry.addData("rise 90%", Double.isNaN(stats.riseS) ? "-" : String.format("%.2f s", stats.riseS));
                telemetry.addData("settle", Double.isNaN(stats.settleS) ? "-" : String.format("%.2f s", stats.settleS));
                telemetry.addData("overshoot", "%.1f°", stats.overshootDeg());
            }
            telemetry.addData("loop", "%.0f ms", dt * 1000);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("target deg", target);
            packet.put("measured deg", deg);
            packet.put("error deg", target - deg);
            packet.put("power", turret.power());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
        turret.setOpenLoop(0);
        turret.periodic();
    }

    private static final class StepStats {
        double startDeg, target, peak, riseS = Double.NaN, settleS = Double.NaN;
        long startNanos;

        void start(double deg, double setpoint) {
            startDeg = deg;
            target = setpoint;
            peak = deg;
            riseS = settleS = Double.NaN;
            startNanos = System.nanoTime();
        }

        void update(double deg) {
            if (Double.isNaN(deg)) return;
            double s = (System.nanoTime() - startNanos) / 1e9, dir = Math.signum(target - startDeg);
            if (dir * deg > dir * peak) peak = deg;
            if (Double.isNaN(riseS) && dir != 0 && dir * (deg - startDeg) >= 0.9 * Math.abs(target - startDeg)) riseS = s;
            boolean in = Math.abs(target - deg) <= TurretConfig.TOLERANCE_DEG;
            if (in && Double.isNaN(settleS)) settleS = s;
            if (!in) settleS = Double.NaN;
        }

        double overshootDeg() {
            return Math.max(0, Math.signum(target - startDeg) * (peak - target));
        }
    }
}
