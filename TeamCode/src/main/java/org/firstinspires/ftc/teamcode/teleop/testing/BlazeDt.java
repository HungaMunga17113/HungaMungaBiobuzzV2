package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.util.control.Toggle;

import dev.anygeneric.blazeftc.DummyPlugOpMode;
import dev.anygeneric.blazeftc.PositionData;
import kotlin.Unit;

/**
 * special:
 *   dpad up         toggle fcd
 *   back (share)    reset pp heading for fcd
 */
@TeleOp(name = "Drivetrain (Blaze)", group = "Main")
public class BlazeDt extends DummyPlugOpMode {
    private static final double MM_PER_IN = 25.4;

    private DcMotorEx frontLeft, backLeft, frontRight, backRight;
    private final double[] lastPower = {Double.NaN, Double.NaN, Double.NaN, Double.NaN};

    private volatile PositionData latest;
    private volatile long latestNanos;
    private volatile int pinpointSamples;

    // raw pinpoint heading at the last reset
    private double zeroH;
    private boolean zeroPending = true;

    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private final Toggle modeButton = new Toggle();
    private final Toggle resetButton = new Toggle();

    // telemetry state
    private final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private final ElapsedTime rateTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private VoltageSensor battery;
    private double voltage = Double.NaN;
    private int loops, lastPinpointSamples;
    private double loopHz, pinpointHz;
    private long resets;

    // last loop's values for telem
    private double rawX, rawY, rawRot, cmdX, cmdY, cmdRot, robotX, robotY;
    private double speed, omega, headingRel;
    private boolean slowDrive, slowTurn, pinpointStale = true;

    @Override
    public void runOpModeInBlaze() {
        initializeBlazeFTC();
        engageMotorAcceleration(); // must come before hardwareMap.get on motors

        frontLeft = motor(Names.FRONT_LEFT, MotorConfig.frontLeft);
        backLeft = motor(Names.BACK_LEFT, MotorConfig.backLeft);
        frontRight = motor(Names.FRONT_RIGHT, MotorConfig.frontRight);
        backRight = motor(Names.BACK_RIGHT, MotorConfig.backRight);
        battery = hardwareMap.voltageSensor.iterator().next();

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Names.PINPOINT);
        pinpoint.resetPosAndIMU();
        engagePinpointAcceleration(pinpoint, data -> {
            latest = data;
            latestNanos = System.nanoTime();
            pinpointSamples++;
            return Unit.INSTANCE;
        });

        while (opModeInInit()) {
            telemetry.addData("mode", fieldCentric ? "FIELD" : "ROBOT");
            telemetry.addLine("keep still, pinpoint is calibrating");
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        runBlazeFTC(0); // hand the hubs to blaze
        telemetryTimer.reset();
        rateTimer.reset();
        voltageTimer.reset();
        zeroPending = true;

        while (opModeIsActive()) {
            updatePose();
            buttons();
            drive();
            telemetry();
        }

        for (DcMotorEx m : motors()) m.setPower(0);
    }

    private DcMotorEx motor(String name, com.qualcomm.robotcore.hardware.DcMotorSimple.Direction dir) {
        DcMotorEx m = hardwareMap.get(DcMotorEx.class, name);
        m.setDirection(dir);
        m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return m;
    }

    private DcMotorEx[] motors() {
        return new DcMotorEx[]{frontLeft, backLeft, frontRight, backRight};
    }

    /** pp -> heading relative to the last reset */
    private void updatePose() {
        PositionData d = latest;
        pinpointStale = d == null
                || (System.nanoTime() - latestNanos) / 1e6 > DrivetrainConfig.PINPOINT_STALE_MS;
        if (d == null) return;

        if (zeroPending) {
            zeroH = d.getDirection();
            zeroPending = false;
        }

        headingRel = wrap(d.getDirection() - zeroH);
        speed = Math.hypot(d.getXVelocity(), d.getYVelocity()) / MM_PER_IN;
        omega = Math.toDegrees(d.getAngVelocity());
    }

    private void buttons() {
        if (modeButton.momentary(gamepad1.dpad_up)) fieldCentric = !fieldCentric;
        if (resetButton.momentary(gamepad1.back)) {
            zeroPending = true;
            resets++;
            gamepad1.rumble(150);
        }
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

        robotX = cmdX;
        robotY = cmdY;
        if (fieldCentric && !pinpointStale) {
            // stick is in the driver's frame, rotate it into the robot's (heading ccw+)
            double cos = Math.cos(headingRel), sin = Math.sin(headingRel);
            robotX = cmdX * cos + cmdY * sin;
            robotY = cmdY * cos - cmdX * sin;
        }

        double denom = Math.max(Math.abs(robotY) + Math.abs(robotX) + Math.abs(cmdRot), 1.0);
        setPower(0, frontLeft, (robotY + robotX + cmdRot) / denom);
        setPower(1, backLeft, (robotY - robotX + cmdRot) / denom);
        setPower(2, frontRight, (robotY - robotX - cmdRot) / denom);
        setPower(3, backRight, (robotY + robotX - cmdRot) / denom);
    }

    /** blaze writes straight to the hub, so skip repeats */
    private void setPower(int i, DcMotorEx m, double power) {
        if (Math.abs(power - lastPower[i]) < 1e-3 && !(power == 0 && lastPower[i] != 0)) return;
        lastPower[i] = power;
        m.setPower(power);
    }

    private double curve(double input) {
        return DrivetrainConfig.DRIVE_CURVE.apply(input, DrivetrainConfig.CURVE_PARAMS);
    }

    private double deadband(double input) {
        double db = DrivetrainConfig.STICK_DB;
        if (Math.abs(input) < db) return 0;
        return (input - Math.signum(input) * db) / (1 - db);
    }

    private static double wrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad < -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    // ---------------------------------------------------------------- telem

    private void telemetry() {
        loops++;
        if (rateTimer.seconds() >= 0.5) {
            int samples = pinpointSamples;
            loopHz = loops / rateTimer.seconds();
            pinpointHz = (samples - lastPinpointSamples) / rateTimer.seconds();
            lastPinpointSamples = samples;
            loops = 0;
            rateTimer.reset();
        }
        if (voltageTimer.seconds() >= 1) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }
        if (telemetryTimer.milliseconds() < DrivetrainConfig.TELEMETRY_MS) return;
        telemetryTimer.reset();

        boolean fcdActive = fieldCentric && !pinpointStale;
        String mode = fieldCentric ? (pinpointStale ? "FIELD (no pinpoint -> robot)" : "FIELD-centric") : "ROBOT-centric";
        double[] p = {lastPower[0], lastPower[1], lastPower[2], lastPower[3]};

        // driver station: only what the driver needs, the rest is on slothboard
        telemetry.addData("mode", (fieldCentric ? "FIELD" : "ROBOT") + (slowDrive || slowTurn ? "  (slow)" : ""));
        telemetry.addData("heading", "%.0f°", Math.toDegrees(headingRel));
        if (pinpointStale) telemetry.addLine("!! PINPOINT LOST - robot-centric only");
        telemetry.update();

        // slothboard: numbers graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("mode", mode);
        packet.put("field centric active", fcdActive);
        packet.put("slow drive", slowDrive);
        packet.put("slow turn", slowTurn);
        packet.put("heading (deg)", Math.toDegrees(headingRel));
        packet.put("angular vel (deg/s)", omega);
        packet.put("speed (in/s)", speed);
        packet.put("stick x raw", rawX);
        packet.put("stick y raw", rawY);
        packet.put("stick rot raw", rawRot);
        packet.put("stick x shaped", cmdX);
        packet.put("stick y shaped", cmdY);
        packet.put("stick rot shaped", cmdRot);
        packet.put("power FL", p[0]);
        packet.put("power BL", p[1]);
        packet.put("power FR", p[2]);
        packet.put("power BR", p[3]);
        packet.put("loop (Hz)", loopHz);
        packet.put("pinpoint (Hz)", pinpointHz);
        packet.put("battery (V)", voltage);
        packet.put("pinpoint resets", resets);
        dashboard.sendTelemetryPacket(packet);
    }
}
