package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.hardware.RobotHardware;

/**
 * dpad left/right  +/- JOG_DEG (ccw+)
 * x / y / b        -90 / 0 / 90
 * lb / rb          MIN_DEG / MAX_DEG
 */
@Config("TurretTester")
@TeleOp(name = "TurretTester", group = "Testers")
public class TurretTester extends LinearOpMode {
    public static double JOG_DEG = 15;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Turret turret = new Turret(robot.optionalCrServo(Names.TURRET), robot.optionalAnalog(Names.TURRET_ENCODER));

        robot.addInitWarnings(telemetry);
        telemetry.addLine("start facing forward, turret holds 0 on start");
        telemetry.update();
        waitForStart();

        double deg = 0;
        while (opModeIsActive()) {
            robot.update();

            if (gamepad1.dpadLeftWasPressed()) deg += JOG_DEG;
            if (gamepad1.dpadRightWasPressed()) deg -= JOG_DEG;
            if (gamepad1.xWasPressed()) deg = -90;
            if (gamepad1.yWasPressed()) deg = 0;
            if (gamepad1.bWasPressed()) deg = 90;
            if (gamepad1.leftBumperWasPressed()) deg = TurretConfig.MIN_DEG;
            if (gamepad1.rightBumperWasPressed()) deg = TurretConfig.MAX_DEG;
            deg = Math.max(TurretConfig.MIN_DEG, Math.min(TurretConfig.MAX_DEG, deg));
            turret.setAngle(Math.toRadians(deg));

            turret.periodic();
            turret.telemetry(telemetry);
            telemetry.update();
        }
    }
}
