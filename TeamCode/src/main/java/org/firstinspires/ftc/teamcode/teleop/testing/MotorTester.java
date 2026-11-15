package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.control.Toggle;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Kind;

import java.util.ArrayList;
import java.util.List;

/**
 * dpad up/down  pick motor
 * hold a / b    run +/- TEST_POWER
 * hold rb       right stick y = power
 * x             flip direction
 * y             zero encoder delta
 */
@Config("MotorTester")
@TeleOp(name = "MotorTester", group = "Testers")
public class MotorTester extends LinearOpMode {
    public static double TEST_POWER = 0.3;

    private final List<String> names = new ArrayList<>();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private final List<String> missing = new ArrayList<>();

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        for (String name : RobotConfig.HARDWARE.names(Kind.MOTOR)) {
            DcMotorEx m = hardwareMap.tryGet(DcMotorEx.class, name);
            if (m == null) {
                missing.add(name);
                continue;
            }
            Direction d = TesterUtil.configuredDirection(name);
            m.setDirection(d == null ? Direction.FORWARD : d);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            names.add(name);
            motors.add(m);
        }

        Toggle up = new Toggle(), down = new Toggle(), flip = new Toggle(), zero = new Toggle();
        ElapsedTime powered = new ElapsedTime();
        int sel = 0, base = 0;
        boolean wasPowered = false;

        telemetry.addLine(motors.size() + " motors from RobotConfig.HARDWARE, nothing moves until you press");
        for (String n : missing) telemetry.addLine("missing on robot: " + n + " (push + activate config?)");
        telemetry.update();
        waitForStart();
        if (motors.isEmpty()) return;
        base = motors.get(0).getCurrentPosition();

        while (opModeIsActive()) {
            int prev = sel;
            if (up.momentary(gamepad1.dpad_up)) sel--;
            if (down.momentary(gamepad1.dpad_down)) sel++;
            sel = TesterUtil.wrapIndex(sel, motors.size());
            DcMotorEx m = motors.get(sel);
            String name = names.get(sel);
            if (sel != prev) {
                motors.get(prev).setPower(0);
                base = m.getCurrentPosition();
            }
            if (flip.momentary(gamepad1.x)) m.setDirection(m.getDirection().inverted());
            if (zero.momentary(gamepad1.y)) base = m.getCurrentPosition();

            double power = 0;
            if (gamepad1.a) power = TEST_POWER;
            else if (gamepad1.b) power = -TEST_POWER;
            else if (gamepad1.right_bumper) power = -gamepad1.right_stick_y;
            m.setPower(power);

            boolean isPowered = Math.abs(power) > 0.05;
            if (isPowered && !wasPowered) powered.reset();
            wasPowered = isPowered;

            double vel = m.getVelocity();
            Direction configured = TesterUtil.configuredDirection(name);
            telemetry.addData("motor", "%s  (%d/%d)", name, sel + 1, motors.size());
            telemetry.addData("port", RobotConfig.HARDWARE.whereIs(name));
            telemetry.addData("type", RobotConfig.HARDWARE.typeOf(name));
            telemetry.addData("direction", "%s %s", m.getDirection(),
                    configured == null ? "(no MotorConfig." + name + ")"
                            : configured == m.getDirection() ? "(= MotorConfig)" : "(MotorConfig says " + configured + ")");
            telemetry.addData("power", "%.2f", power);
            telemetry.addData("encoder", "%d  (delta %d)", m.getCurrentPosition(), m.getCurrentPosition() - base);
            telemetry.addData("velocity", "%.0f ticks/s", vel);
            telemetry.addData("current", "%.2f A", m.getCurrent(CurrentUnit.AMPS));
            if (isPowered && powered.milliseconds() > 300) telemetry.addLine(verdict(power, vel));
            telemetry.addLine();
            for (int i = 0; i < names.size(); i++) telemetry.addLine((i == sel ? "> " : "   ") + names.get(i));
            telemetry.update();
        }
        for (DcMotorEx m : motors) m.setPower(0);
    }

    private static String verdict(double power, double vel) {
        if (Math.abs(vel) < 5) return "!! no encoder counts: encoder cable unplugged or on another port?";
        if (Math.signum(vel) != Math.signum(power)) return "!! encoder counts against the power: motor/encoder wiring reversed";
        return "ok: encoder follows power";
    }
}
