package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.util.control.Toggle;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumMath;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

/**
 * dpad up       toggle fcd
 * back (share)  reset heading for fcd
 */
@TeleOp(name = "Drivetrain", group = "Main")
public class Dt extends OpMode {
    private RobotHardware robot;
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private PinpointLocalizer pinpoint;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private boolean zeroPending = true;
    private final Toggle modeButton = new Toggle(), resetButton = new Toggle();
    private long resets;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        robot = new RobotHardware(hardwareMap);
        frontLeft = robot.motor(Names.FRONT_LEFT);
        backLeft = robot.motor(Names.BACK_LEFT);
        frontRight = robot.motor(Names.FRONT_RIGHT);
        backRight = robot.motor(Names.BACK_RIGHT);
        pinpoint = robot.pinpoint();
    }

    @Override
    public void init_loop() {
        robot.update();
        robot.addInitWarnings(telemetry);
        telemetry.addData("mode", fieldCentric ? "FIELD" : "ROBOT");
    }

    @Override
    public void start() {
        loopTimer.reset();
    }

    @Override
    public void loop() {
        robot.update();
        boolean stale = robot.pinpointStale();
        if (zeroPending && !stale) {
            pinpoint.setHeading(0);
            zeroPending = false;
        }
        double heading = pinpoint.pose().heading();

        if (modeButton.momentary(gamepad1.dpad_up)) fieldCentric = !fieldCentric;
        if (resetButton.momentary(gamepad1.back)) {
            zeroPending = true;
            resets++;
            gamepad1.rumble(150);
        }

        // +y forward, +x right, +rot clockwise
        double rawX = deadband(gamepad1.left_stick_x);
        double rawY = -deadband(gamepad1.left_stick_y);
        double rawRot = deadband(gamepad1.right_stick_x);
        boolean slowDrive = gamepad1.left_stick_button, slowTurn = gamepad1.right_stick_button;
        double scale = DrivetrainConfig.AIM_TURN_SCALE;
        double x = slowDrive ? scale * rawX : curve(rawX);
        double y = slowDrive ? scale * rawY : curve(rawY);
        double rot = slowTurn ? scale * rawRot : DrivetrainConfig.TURN_SPEED * curve(rawRot);

        double[] r = fieldCentric && !stale ? MecanumMath.fieldToRobot(x, y, heading) : new double[]{x, y};
        double[] p = MecanumMath.mix(r[1], r[0], rot);
        frontLeft.setPower(p[0]);
        backLeft.setPower(p[1]);
        frontRight.setPower(p[2]);
        backRight.setPower(p[3]);

        telemetry.addData("mode", (fieldCentric ? "FIELD" : "ROBOT") + (slowDrive || slowTurn ? "  (slow)" : ""));
        telemetry.addData("heading", "%.0f°", Math.toDegrees(heading));
        if (stale) telemetry.addLine("!! PINPOINT LOST - robot-centric only");

        Velocity v = pinpoint.velocity();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("field centric active", fieldCentric && !stale);
        packet.put("angular vel (deg/s)", Math.toDegrees(v.omega));
        packet.put("speed (in/s)", v.linSpeed());
        packet.put("stick x / y / rot raw", String.format("%.2f %.2f %.2f", rawX, rawY, rawRot));
        packet.put("power FL", p[0]);
        packet.put("power BL", p[1]);
        packet.put("power FR", p[2]);
        packet.put("power BR", p[3]);
        packet.put("pinpoint resets", resets);
        packet.put("loop (ms)", loopTimer.milliseconds());
        packet.put("battery (V)", robot.voltage());
        loopTimer.reset();
        dashboard.sendTelemetryPacket(packet);
    }

    private static double curve(double input) {
        return DrivetrainConfig.DRIVE_CURVE.apply(input, DrivetrainConfig.CURVE_PARAMS);
    }

    private static double deadband(double input) {
        return MecanumMath.deadband(input, DrivetrainConfig.STICK_DB);
    }
}
