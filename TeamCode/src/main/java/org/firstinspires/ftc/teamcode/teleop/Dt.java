package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
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

    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private boolean zeroPending = true;
    private long resets;
    private final ElapsedTime loopTimer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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

        if (gamepad1.dpadUpWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.backWasPressed()) {
            zeroPending = true;
            resets++;
            gamepad1.rumble(150);
        }

        // +y forward, +x right, +rot clockwise
        boolean slowDrive = gamepad1.left_stick_button, slowTurn = gamepad1.right_stick_button;
        double x = shape(gamepad1.left_stick_x, slowDrive, 1);
        double y = shape(-gamepad1.left_stick_y, slowDrive, 1);
        double rot = shape(gamepad1.right_stick_x, slowTurn, DrivetrainConfig.TURN_SPEED);

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
        telemetry.addData("speed (in/s)", v.linSpeed());
        telemetry.addData("angular vel (deg/s)", Math.toDegrees(v.omega));
        telemetry.addData("power FL BL FR BR", "%.2f %.2f %.2f %.2f", p[0], p[1], p[2], p[3]);
        telemetry.addData("pinpoint resets", resets);
        telemetry.addData("loop (ms)", loopTimer.milliseconds());
        telemetry.addData("battery (V)", robot.voltage());
        loopTimer.reset();
    }

    /** deadband, then the slow scale or the curve (times scale) */
    private static double shape(double input, boolean slow, double scale) {
        double v = MecanumMath.deadband(input, DrivetrainConfig.STICK_DB);
        return slow ? DrivetrainConfig.SLOW_SCALE * v : scale * DrivetrainConfig.DRIVE_CURVE.apply(v, DrivetrainConfig.CURVE_PARAMS);
    }
}
