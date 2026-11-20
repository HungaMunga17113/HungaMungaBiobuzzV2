package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.TransferConfig;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.hardware.MotorGroup;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

/**
 * a           shooter on / off (targetRpm)
 * dpad up/dn  targetRpm +/- RPM_STEP
 * l trigger   intake in, l bumper out
 * r bumper    shoot one ball (waits for speed)
 */
@Config("ShooterTester")
@TeleOp(name = "ShooterTester", group = "Testers")
public class ShooterTester extends LinearOpMode {
    public static double RPM_STEP = 100;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Intake intake = new Intake(robot.optionalMotor(Names.INTAKE));
        Transfer transfer = new Transfer(robot.optionalServo(Names.TRANSFER));
        DcMotorEx[] fly = new DcMotorEx[Names.FLYWHEEL.length];
        for (int i = 0; i < fly.length; i++) fly[i] = robot.optionalMotor(Names.FLYWHEEL[i]);
        Shooter shooter = new Shooter(new MotorGroup(fly), robot::voltage);

        transfer.set(TransferConfig.REST_POS);
        robot.addInitWarnings(telemetry);
        telemetry.addLine("a spins the flywheel, keep clear");
        telemetry.update();
        waitForStart();

        boolean on = false, queued = false;
        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.aWasPressed()) on = !on;
            if (gamepad1.dpadUpWasPressed()) ShooterConfig.targetRpm += RPM_STEP;
            if (gamepad1.dpadDownWasPressed()) ShooterConfig.targetRpm = Math.max(0, ShooterConfig.targetRpm - RPM_STEP);
            shooter.setTarget(on ? ShooterConfig.targetRpm : 0);

            intake.set(gamepad1.left_trigger > 0.3 ? IntakeConfig.IN_POWER
                    : gamepad1.left_bumper ? IntakeConfig.OUT_POWER : 0);

            if (gamepad1.rightBumperWasPressed() && on) queued = true;
            if (!on) queued = false;
            if (queued && shooter.atSpeed() && !transfer.isFeeding()) {
                transfer.startFeed();
                queued = false;
            }

            shooter.periodic();
            intake.periodic();
            transfer.periodic();

            shooter.telemetry(telemetry);
            intake.telemetry(telemetry);
            transfer.telemetry(telemetry);
            if (queued) telemetry.addLine("shot queued");
            telemetry.update();
        }
        shooter.setTarget(0);
        shooter.periodic();
        intake.set(0);
    }
}
