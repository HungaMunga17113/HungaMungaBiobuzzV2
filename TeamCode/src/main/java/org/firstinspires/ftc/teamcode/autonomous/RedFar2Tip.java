package org.firstinspires.ftc.teamcode.autonomous;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConfig.AutoConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.TransferConfig;
import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Hive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

import java.util.Arrays;
import java.util.List;

/** preload, loading zone, top flower + shoot, left flower, shoot, park in the garden. turret tracks the hive throughout */
@Autonomous(name = "Red Far - 2 Tip", group = "Autonomous")
public class RedFar2Tip extends LinearOpMode {
    private RobotHardware robot;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private Flower flower;
    private List<Subsystem> subsystems;
    private Hive hive;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58, 8.25, 90);
    private final Pose shoot1 = poseFactory.of(58, 13.5, 90);
    private final Pose intake1 = poseFactory.of(8.6, 8, 180);
    private final Pose intake1Control1 = poseFactory.of(31, 14, 0);
    private final Pose intake1Control2 = poseFactory.of(36, 10.5, 0);
    private final Pose intake1Control3 = poseFactory.of(18.8, 6.8, 0);
    private final Pose switch1Start = poseFactory.of(8.6, 8, 180);
    private final Pose switch1 = poseFactory.of(25, 94, 150);
    private final Pose switch1Control1 = poseFactory.of(23.5, 91.5, 0);
    private final Pose flower1andshootStart = poseFactory.of(25, 94, 150);
    private final Pose flower1andshoot = poseFactory.of(46.5, 130, 90);
    private final Pose flower1andshootControl1 = poseFactory.of(30, 100, 0);
    private final Pose flower1andshootControl2 = poseFactory.of(30, 108, 0);
    private final Pose switch2Start = poseFactory.of(46.5, 130, 90);
    private final Pose switch2 = poseFactory.of(21.75, 47, 180);
    private final Pose switch2Control1 = poseFactory.of(24.5, 100, 0);
    private final Pose flower2 = poseFactory.of(11.75, 47.25, 178.5679);
    private final Pose shoot3Start = poseFactory.of(11.75, 47.25, 180);
    private final Pose shoot3 = poseFactory.of(34.4, 25.5007, 90);
    private final Pose shoot3Control1 = poseFactory.of(13, 33.5, 0);
    private final Pose shoot3Control2 = poseFactory.of(20, 26.7, 0);
    private final Pose parkStart = poseFactory.of(34.4, 25.5007, 90);
    private final Pose park = poseFactory.of(9.5, 96.5, 90);
    private final Pose parkControl1 = poseFactory.of(11.5, 29, 0);

    /** must not touch the turret, the separate track command owns it */
    public Command autoRoutine() {
        return sequential(
                parallel(
                        drive.follow(shoot1()),
                        shooter.spinUp(Shooter.rpmFor(null))
                ),
                volley(),
                tip(),

                deadline(
                        sequential(drive.follow(intake1()), waitMs(AutoConfig.INTAKE_DWELL_MS), drive.follow(switch1())),
                        intake.in()
                ),

                drive.follow(flower1andshoot()),
                flowerGrab(),
                volley(),
                tip(),

                drive.follow(switch2()),
                drive.follow(flower2()),
                flowerGrab(),

                drive.follow(shoot3()),
                volley(),

                parallel(
                        drive.follow(park()),
                        shooter.stop(),
                        intake.stop(),
                        flower.rest()
                )
        );
    }

    private Command volley() {
        return race(
                deadline(
                        repeat(sequential(turret.waitUntilOnTarget(), shooter.waitUntilReady(), transfer.feed()), () -> AutoConfig.VOLLEY_BALLS),
                        intake.in()
                ),
                waitMs(AutoConfig.VOLLEY_TIMEOUT_MS)
        );
    }
    private Command tip() {
        return instant(() -> hive.tipped());
    }

    private Command flowerGrab() {
        return sequential(
                flower.feed(),
                deadline(waitMs(AutoConfig.FLOWER_MS), intake.in()),
                flower.rest()
        );
    }

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Scheduler.reset();
        Alliance.selected = Alliance.RED;
        hive = new Hive(Alliance.RED);

        robot = new RobotHardware(hardwareMap);
        drive = new Drivetrain(robot.follower());
        intake = new Intake(robot.optionalMotor(Names.INTAKE));
        transfer = new Transfer(robot.optionalServo(Names.TRANSFER));
        flower = new Flower(robot.optionalServo(Names.FLOWER));
        turret = new Turret(robot.optionalCrServo(Names.TURRET), robot.optionalAnalog(Names.TURRET_ENCODER));

        DcMotorEx[] fly = new DcMotorEx[Names.FLYWHEEL.length];
        for (int i = 0; i < fly.length; i++) fly[i] = robot.optionalMotor(Names.FLYWHEEL[i]);
        shooter = new Shooter(new MotorGroup(fly), robot::voltage);
        subsystems = Arrays.asList(drive, intake, transfer, shooter, turret, flower);

        drive.relocalize(start);
        transfer.set(TransferConfig.REST_POS);
        flower.feed(false);
        turret.setAngle(0);

        while (opModeInInit()) {
            robot.update();
            telemetry.addLine("red far, 2 tip. start pose " + start);
            robot.addInitWarnings(telemetry);
            telemetry.update();
        }
        if (isStopRequested()) return;

        drive.relocalize(start);
        schedule(turret.track(drive::pose, () -> drive.follower().velocity(), hive::targetX, hive::targetY));
        schedule(autoRoutine());

        while (opModeIsActive()) {
            robot.update();
            Scheduler.execute();

            TurretAim.Solution aim = turret.solution();
            if (shooter.on()) shooter.setTarget(Shooter.rpmFor(aim));

            for (Subsystem s : subsystems) s.periodic();
            for (Subsystem s : subsystems) s.telemetry(telemetry);
            telemetry.addData("hive", "%s cell up", hive.up());
            telemetry.update();
        }
    }

    public Path shoot1() {
        return Paths.line(start, shoot1).linear(start, shoot1);
    }

    public Path intake1() {
        return Paths.curve(shoot1, intake1Control1, intake1Control2, intake1Control3, intake1).constant(intake1);
    }

    public Path switch1() {
        return Paths.curve(switch1Start, switch1Control1, switch1).linear(switch1Start, switch1);
    }

    public Path flower1andshoot() {
        return Paths.curve(flower1andshootStart, flower1andshootControl1, flower1andshootControl2, flower1andshoot).linear(flower1andshootStart, flower1andshoot);
    }

    public Path switch2() {
        return Paths.curve(switch2Start, switch2Control1, switch2).linear(switch2Start, switch2);
    }

    public Path flower2() {
        return Paths.line(switch2, flower2).tangent();
    }

    public Path shoot3() {
        return Paths.curve(shoot3Start, shoot3Control1, shoot3Control2, shoot3).linear(shoot3Start, shoot3);
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
