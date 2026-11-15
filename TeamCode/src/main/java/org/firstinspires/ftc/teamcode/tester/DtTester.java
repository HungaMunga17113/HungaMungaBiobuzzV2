package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;

/**
 * single wheel (forward):  lb fl  rb fr  lt bl  rt br
 * whole robot:             dpad = drive/strafe, x / b = turn ccw / cw
 * robot on blocks first
 */
@Config("DtTester")
@TeleOp(name = "DtTester", group = "Testers")
public class DtTester extends LinearOpMode {
    public static double TEST_POWER = 0.3;

    private static final String[] LABELS = {"frontLeft", "backLeft", "frontRight", "backRight"};

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        DcMotorEx[] m = {
                motor(Names.FRONT_LEFT, MotorConfig.frontLeft),
                motor(Names.BACK_LEFT, MotorConfig.backLeft),
                motor(Names.FRONT_RIGHT, MotorConfig.frontRight),
                motor(Names.BACK_RIGHT, MotorConfig.backRight)
        };

        telemetry.addLine("put the robot on blocks");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double[] p = new double[4];
            String test = "none";
            double s = TEST_POWER;
            if (gamepad1.left_bumper) { p[0] = s; test = "frontLeft only, should spin forward"; }
            else if (gamepad1.right_bumper) { p[2] = s; test = "frontRight only, should spin forward"; }
            else if (gamepad1.left_trigger > 0.5) { p[1] = s; test = "backLeft only, should spin forward"; }
            else if (gamepad1.right_trigger > 0.5) { p[3] = s; test = "backRight only, should spin forward"; }
            else if (gamepad1.dpad_up) { p = mix(s, 0, 0); test = "forward: all +"; }
            else if (gamepad1.dpad_down) { p = mix(-s, 0, 0); test = "backward: all -"; }
            else if (gamepad1.dpad_right) { p = mix(0, s, 0); test = "strafe right: fl+ bl- fr- br+"; }
            else if (gamepad1.dpad_left) { p = mix(0, -s, 0); test = "strafe left: fl- bl+ fr+ br-"; }
            else if (gamepad1.b) { p = mix(0, 0, s); test = "turn cw: left +, right -"; }
            else if (gamepad1.x) { p = mix(0, 0, -s); test = "turn ccw: left -, right +"; }

            telemetry.addData("test", test);
            for (int i = 0; i < 4; i++) {
                if (m[i] == null) {
                    telemetry.addData(LABELS[i], "missing");
                    continue;
                }
                m[i].setPower(p[i]);
                double vel = m[i].getVelocity();
                String warn = Math.abs(p[i]) > 0.05 && Math.abs(vel) < 5 ? "  !! no encoder counts" : "";
                telemetry.addData(LABELS[i], "cmd %+.2f  enc %s %6.0f t/s  (%s)%s",
                        p[i], TesterUtil.sign(vel, 5), vel, m[i].getDirection(), warn);
            }
            telemetry.addLine();
            telemetry.addLine("wheel spins the wrong way: flip it in MotorConfig");
            telemetry.addLine("wrong wheel spins: ports swapped, fix RobotConfig.HARDWARE or the wiring");
            telemetry.addLine("strafe drifts diagonally: check roller orientation (x from above)");
            telemetry.update();
        }
        for (DcMotorEx mm : m) if (mm != null) mm.setPower(0);
    }

    private DcMotorEx motor(String name, Direction d) {
        DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
        if (m == null) return null;
        m.setDirection(d);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        return m;
    }

    /** fl, bl, fr, br */
    private static double[] mix(double forward, double strafe, double turn) {
        double d = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(turn), 1);
        return new double[]{
                (forward + strafe + turn) / d,
                (forward - strafe + turn) / d,
                (forward - strafe - turn) / d,
                (forward + strafe - turn) / d
        };
    }
}
