package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Velocity;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.util.control.Toggle;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumMath;
import org.firstinspires.ftc.teamcode.util.opmode.BlazeOpMode;

/**
 * dpad up       toggle fcd
 * back (share)  reset heading for fcd
 */
@TeleOp(name = "Drivetrain (Blaze)", group = "Main")
public class BlazeDt extends BlazeOpMode {
    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private final double[] lastPower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private boolean zeroPending = true;
    private final Toggle modeButton = new Toggle();
    private final Toggle resetButton = new Toggle();
    private long resets;

    private double rawX, rawY, rawRot, cmdX, cmdY, cmdRot, heading;
    private boolean slowDrive, slowTurn;

    @Override
    protected void onInit() {
        frontLeft = motor(Names.FRONT_LEFT, MotorConfig.frontLeft);
        backLeft = motor(Names.BACK_LEFT, MotorConfig.backLeft);
        frontRight = motor(Names.FRONT_RIGHT, MotorConfig.frontRight);
        backRight = motor(Names.BACK_RIGHT, MotorConfig.backRight);
        pinpoint();
    }

    @Override
    protected void onInitLoop() {
        telemetry.addData("mode", fieldCentric ? "FIELD" : "ROBOT");
    }

    @Override
    protected void onLoop() {
        if (zeroPending && pinpoint().hasData()) {
            pinpoint().setHeading(0);
            zeroPending = false;
        }
        heading = pinpoint().pose().heading();

        if (modeButton.momentary(gamepad1.dpad_up)) fieldCentric = !fieldCentric;
        if (resetButton.momentary(gamepad1.back)) {
            zeroPending = true;
            resets++;
            gamepad1.rumble(150);
        }
        drive();
    }

    private void drive() {
        // +y forward, +x right, +rot clockwise
        rawX = deadband(gamepad1.left_stick_x);
        rawY = -deadband(gamepad1.left_stick_y);
        rawRot = deadband(gamepad1.right_stick_x);

        slowDrive = gamepad1.left_stick_button;
        slowTurn = gamepad1.right_stick_button;
        double scale = DrivetrainConfig.AIM_TURN_SCALE;
        cmdX = slowDrive ? scale * rawX : curve(rawX);
        cmdY = slowDrive ? scale * rawY : curve(rawY);
        cmdRot = slowTurn ? scale * rawRot : DrivetrainConfig.TURN_SPEED * curve(rawRot);

        double[] r = fieldCentric && !pinpointStale()
                ? MecanumMath.fieldToRobot(cmdX, cmdY, heading)
                : new double[]{cmdX, cmdY};
        double[] p = MecanumMath.mix(r[1], r[0], cmdRot);
        setPower(0, frontLeft, p[0]);
        setPower(1, backLeft, p[1]);
        setPower(2, frontRight, p[2]);
        setPower(3, backRight, p[3]);
    }

    private void setPower(int i, DcMotorEx m, double power) {
        if (Math.abs(power - lastPower[i]) < 1e-3 && !(power == 0 && lastPower[i] != 0)) return;
        lastPower[i] = power;
        m.setPower(power);
    }

    private double curve(double input) {
        return DrivetrainConfig.DRIVE_CURVE.apply(input, DrivetrainConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        return MecanumMath.deadband(input, DrivetrainConfig.STICK_DB);
    }

    @Override
    protected void onTelemetry(Telemetry ds, TelemetryPacket dash) {
        boolean stale = pinpointStale();
        ds.addData("mode", (fieldCentric ? "FIELD" : "ROBOT") + (slowDrive || slowTurn ? "  (slow)" : ""));
        ds.addData("heading", "%.0f°", Math.toDegrees(heading));
        if (stale) ds.addLine("!! PINPOINT LOST - robot-centric only");

        Velocity v = pinpoint().velocity();
        dash.put("mode", fieldCentric ? (stale ? "FIELD (no pinpoint -> robot)" : "FIELD-centric") : "ROBOT-centric");
        dash.put("field centric active", fieldCentric && !stale);
        dash.put("slow drive", slowDrive);
        dash.put("slow turn", slowTurn);
        dash.put("angular vel (deg/s)", Math.toDegrees(v.omega));
        dash.put("speed (in/s)", v.linSpeed());
        dash.put("stick x raw", rawX);
        dash.put("stick y raw", rawY);
        dash.put("stick rot raw", rawRot);
        dash.put("stick x shaped", cmdX);
        dash.put("stick y shaped", cmdY);
        dash.put("stick rot shaped", cmdRot);
        dash.put("power FL", lastPower[0]);
        dash.put("power BL", lastPower[1]);
        dash.put("power FR", lastPower[2]);
        dash.put("power BR", lastPower[3]);
        dash.put("pinpoint resets", resets);
    }
}
