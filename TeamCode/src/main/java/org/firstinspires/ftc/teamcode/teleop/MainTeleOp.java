package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.Hive;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.firstinspires.ftc.teamcode.util.drivetrain.MecanumMath;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

import java.util.Arrays;
import java.util.List;

/**
 * init_loop: x = BLUE, b = RED on either gp, gp2 d_up / d_down = high / low cell up
 *
 * gp1 (drive)
 *   sticks         drive, btns = slow
 *   d_up           toggle fcd
 *   back (share)   heading reset: robot facing away from u
 *   d_down         relocalize: robot squared into our corner (Field.corner), enables auto-aim
 *   left t+b       intake in / out
 *   r trigger      shooter on / off
 *   r bumper       shoot one ball once at speed, release cancels
 *   y              flower mech toggle
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
    private Flower flower;
    private List<Subsystem> subsystems;

    private Alliance alliance;
    private Hive hive;
    private Hive.Cell hiveUp;


    // comp(only) reminders
    private static final double[] RUMBLE_S = {60, 100}; // 1:00 + 0:20 left
    private int rumbles;


    private boolean fieldCentric = DrivetrainConfig.useFieldCentric;
    private double pendingHeading = Double.NaN;
    private boolean localized, autoAim = true, shooterOn, shotQueued;

    @Override
    public void init() {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotHardware(hardwareMap);
        hiveUp = Hive.autoEndUp;
        Hive.autoEndUp = null;
        drive = new Drivetrain(robot.follower());
        intake = robot.intake();
        transfer = new Transfer(robot.optionalServo(Names.TRANSFER));
        flower = new Flower(robot.optionalServo(Names.FLOWER));

        DcMotorEx[] fly = new DcMotorEx[Names.FLYWHEEL.length];
        for (int i = 0; i < fly.length; i++) fly[i] = robot.optionalMotor(Names.FLYWHEEL[i]);

        shooter = new Shooter(new MotorGroup(fly), robot::voltage);
        turret = new Turret(robot.optionalCrServo(Names.TURRET), robot.optionalAnalog(Names.TURRET_ENCODER));

        subsystems = Arrays.asList(drive, intake, transfer, shooter, turret, flower);
    }

    @Override
    public void init_loop() {
        robot.update();

        if (gamepad1.x || gamepad2.x) Alliance.selected = Alliance.BLUE;
        if (gamepad1.b || gamepad2.b) Alliance.selected = Alliance.RED;
        if (gamepad2.dpadUpWasPressed()) hiveUp = Hive.Cell.HIGH;
        if (gamepad2.dpadDownWasPressed()) hiveUp = Hive.Cell.LOW;
        hiveLine(hiveUp != null ? hiveUp : Field.startUp(Alliance.selected));

        telemetry.addLine("ALLIANCE: " + Alliance.selected + "   (x = blue, b = red)");
        telemetry.addLine("square into corner + dpad down (gp1) to localize");
        robot.addInitWarnings(telemetry);
    }

    @Override
    public void start() {
        alliance = Alliance.selected;
        hive = new Hive(alliance, hiveUp != null ? hiveUp : Field.startUp(alliance));
        resetRuntime();

        flower.feed(false);
        transfer.set(TransferConfig.REST_POS);
        turret.setAngle(0);
        pendingHeading = Double.isNaN(Drivetrain.autoEndHeading) ? alliance.driverForward : Drivetrain.autoEndHeading;
        Drivetrain.autoEndHeading = Double.NaN;
    }

    @Override
    public void loop() {
        robot.update();

        if (!Double.isNaN(pendingHeading) && !robot.pinpointStale()) {
            drive.follower().setHeading(pendingHeading);
            pendingHeading = Double.NaN;
        }

        /// gp1
        // localization
        if (gamepad1.dpadUpWasPressed()) fieldCentric = !fieldCentric;
        if (gamepad1.backWasPressed()) {
            pendingHeading = alliance.driverForward;
            gamepad1.rumble(150);
        }
        if (gamepad1.dpadDownWasPressed()) {
            drive.relocalize(Field.corner(alliance));
            pendingHeading = Double.NaN;
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
        handleFlower();

        // gp2
        if (gamepad2.yWasPressed()) autoAim = !autoAim;
        boolean tracking = autoAim && localized && !RobotConfig.STATIC_AIM;
        TurretAim.Solution aim = tracking
                ? turret.aimAt(drive.pose(), drive.follower().velocity(), hive.targetX(), hive.targetY())
                : null;
        if (!tracking) turret.setAngle(0);
        shooter.setTarget(shooterOn ? Shooter.rpmFor(aim) : 0);

        // tip notifier
        if (gamepad2.dpadLeftWasPressed()) {
            hive.tipped();
            gamepad2.rumble(200);
        }
        if (rumbles < RUMBLE_S.length && getRuntime() >= RUMBLE_S[rumbles] && RobotConfig.COMPETITION) {
            gamepad1.rumbleBlips(3);
            gamepad2.rumbleBlips(3);
            rumbles++;
        }

        for (Subsystem s : subsystems) s.periodic();
        hiveLine(hive.up());
        telemetry.addData("alliance", alliance);
        telemetry.addData("drive", (fieldCentric ? "FIELD" : "ROBOT") + (robot.pinpointStale() ? "  !! PINPOINT LOST" : ""));
        telemetry.addData("localized", localized ? "yes" : "NO - square into corner, gp1 dpad down");
        telemetry.addData("hive", "%s cell up, %d tips called", hive.up(), hive.tips());
        telemetry.addData("auto-aim", RobotConfig.STATIC_AIM ? "STATIC_AIM (forward, fixed rpm)" : autoAim ? (tracking ? "tracking" : "waiting for localize") : "off");

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
        if (gamepad1.rightTriggerWasPressed()) shooterOn = !shooterOn;

        // one ball, fires once the flywheel is at speed
        if (gamepad1.rightBumperWasPressed()) shotQueued = true;
        if (!gamepad1.right_bumper || !shooterOn) shotQueued = false;

        if (shotQueued && shooter.atSpeed() && !transfer.isFeeding()) {
            transfer.startFeed();
            intake.fired();
            shotQueued = false;
        }
    }
    private void handleIntake() {
        intake.set((gamepad1.left_trigger  > 0.3) ? IntakeConfig.IN_POWER
                : (gamepad1.left_bumper) ? IntakeConfig.OUT_POWER : 0);
    }
    private void handleFlower() { if (gamepad1.yWasPressed()) flower.toggle(); }

    private void hiveLine(Hive.Cell up) {
        telemetry.addLine("<h2>HIVE: " + up + " CELL UP</h2>");
    }

    /** heading relative to "away from our drivers", NaN = robot-centric */
    private double driverHeading() {
        if (!fieldCentric || robot.pinpointStale()) return Double.NaN;
        return drive.pose().heading() - alliance.driverForward;
    }
}
