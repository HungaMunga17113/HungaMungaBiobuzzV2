package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig.DrivetrainConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.TransferConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Hive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumMath;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

import java.util.Arrays;
import java.util.List;

/**
 * init_loop: x = BLUE, b = RED on either gp
 *
 * gp1 (drive)
 *   sticks         drive, btns = slow
 *   d_up           toggle fcd
 *   back (share)   heading reset: robot facing away from u
 *   d_down         relocalize: robot squared into our corner (Field.corner), enables auto-aim
 *   left t+b       intake in / out
 *   r trigger      shooter on / off
 *   r bumper       shoot one ball (waits for the flywheel to be at speed)
 *
 * gp2 (operator)
 *   y                 auto-aim on / off (off = turret forward)
 *   d_left            hive tip (swaps the aim cell, press again to undo)
 */

@TeleOp(name = "Main", group = "Main")
public class MainTeleOp extends OpMode {
    private RobotHardware robot;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private List<Subsystem> subsystems;

    private Alliance alliance;
    private Hive hive;

    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private boolean headingPending = true, localized, autoAim = true, shooterOn, shotQueued;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        drive = new Drivetrain(robot.follower());
        intake = new Intake(robot.optionalMotor(Names.INTAKE));
        transfer = new Transfer(robot.hw.orStub(Servo.class, Names.TRANSFER));

        DcMotorEx[] fly = new DcMotorEx[Names.FLYWHEEL.length];
        for (int i = 0; i < fly.length; i++) fly[i] = robot.optionalMotor(Names.FLYWHEEL[i]);

        shooter = new Shooter(new MotorGroup(fly), robot::voltage);
        turret = new Turret();
        subsystems = Arrays.asList(drive, intake, transfer, shooter, turret);
    }

    @Override
    public void init_loop() {
        robot.update();

        if (gamepad1.x || gamepad2.x) Alliance.selected = Alliance.BLUE;
        if (gamepad1.b || gamepad2.b) Alliance.selected = Alliance.RED;

        telemetry.addLine("ALLIANCE: " + Alliance.selected + "   (x = blue, b = red)");
        telemetry.addLine("after start: square into corner + dpad down (gp1) to localize");
        robot.addInitWarnings(telemetry);
    }

    @Override
    public void start() {
        alliance = Alliance.selected;
        hive = new Hive(alliance);

        transfer.set(TransferConfig.REST_POS);
        turret.setAngle(0);
    }

    @Override
    public void loop() {
        robot.update();

        if (headingPending && !robot.pinpointStale()) {
            drive.follower().setHeading(alliance.driverForward);
            headingPending = false;
        }

        /// gp1
        // localization
        if (gamepad1.dpadUpWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.backWasPressed()) {
            headingPending = true;
            gamepad1.rumble(150);
        }
        if (gamepad1.dpadDownWasPressed()) {
            drive.relocalize(Field.corner(alliance));
            headingPending = false;
            localized = true;
            gamepad1.rumble(300);
        }

        // drive: +right, +forward (away from the driver), +cw. stick buttons = slow
        boolean slowDrive = gamepad1.left_stick_button, slowTurn = gamepad1.right_stick_button;
        drive.drive(shape(gamepad1.left_stick_x, slowDrive, 1),
                shape(-gamepad1.left_stick_y, slowDrive, 1),
                shape(gamepad1.right_stick_x, slowTurn, DrivetrainConfig.TURN_SPEED),
                driverHeading());

        handleIntake();
        handleShooter();

        // gp2
        if (gamepad2.yWasPressed()) autoAim = !autoAim;
        boolean tracking = autoAim && localized;
        TurretAim.Solution aim = tracking
                ? turret.aimAt(drive.pose(), drive.follower().velocity(), hive.targetX(), hive.targetY())
                : null;
        if (!tracking) turret.setAngle(0);
        shooter.setTarget(shooterOn ? Shooter.rpmFor(aim != null ? aim.distance : Double.NaN) : 0);

        if (gamepad2.dpadLeftWasPressed()) {
            hive.tipped();
            gamepad2.rumble(200);
        }

        for (Subsystem s : subsystems) s.periodic();
        telemetry.addData("alliance", alliance);
        telemetry.addData("drive", (fieldCentric ? "FIELD" : "ROBOT") + (robot.pinpointStale() ? "  !! PINPOINT LOST" : ""));
        telemetry.addData("localized", localized ? "yes" : "NO - square into corner, gp1 dpad down");
        telemetry.addData("hive", "%s cell up, %d tips called", hive.up(), hive.tips());
        telemetry.addData("auto-aim", autoAim ? (tracking ? "tracking" : "waiting for localize") : "off");

        for (Subsystem s : subsystems) s.telemetry(telemetry);
        Pose p = drive.pose();
        telemetry.addData("pose", "%.1f, %.1f  %.0f°", p.x(), p.y(), Math.toDegrees(p.heading()));
        telemetry.addData("target", "%.1f, %.1f", hive.targetX(), hive.targetY());
        if (aim != null) {
            telemetry.addData("aim wanted (deg)", Math.toDegrees(aim.wantedRad));
            telemetry.addData("aim distance (in)", aim.distance);
        }
        //telemetry.addData("battery (V)", robot.voltage());
    }

    /** deadband, then the slow scale or the curve (times scale) */
    private static double shape(double input, boolean slow, double scale) {
        double v = MecanumMath.deadband(input, DrivetrainConfig.STICK_DB);
        return slow ? DrivetrainConfig.AIM_TURN_SCALE * v : scale * DrivetrainConfig.DRIVE_CURVE.apply(v, DrivetrainConfig.CURVE_PARAMS);
    }

    /// helpers
    private void handleShooter() {
        // shooter
        if (gamepad1.right_trigger > 0.3) shooterOn = !shooterOn;

        // one ball, fires once the flywheel is at speed
        if (gamepad1.right_bumper && shooterOn) shotQueued = true;
        if (!shooterOn) shotQueued = false;

        if (shotQueued && shooter.atSpeed() && !transfer.isFeeding()) {
            transfer.startFeed();
            shotQueued = false;
        }
    }
    private void handleIntake() {
        intake.set((gamepad1.left_trigger  > 0.3) ? IntakeConfig.IN_POWER
                : (gamepad1.left_bumper) ? IntakeConfig.OUT_POWER : 0);
    }

    /** heading relative to "away from our drivers", NaN = robot-centric */
    private double driverHeading() {
        if (!fieldCentric || robot.pinpointStale()) return Double.NaN;
        return drive.pose().heading() - alliance.driverForward;
    }
}
