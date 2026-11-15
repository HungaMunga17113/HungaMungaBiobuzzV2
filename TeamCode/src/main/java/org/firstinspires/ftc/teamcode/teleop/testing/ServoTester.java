package org.firstinspires.ftc.teamcode.teleop.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.util.control.Toggle;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Kind;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Type;

import java.util.ArrayList;
import java.util.List;

/**
 * dpad up/down     pick servo
 * dpad left/right  -/+ 0.01
 * lb / rb          -/+ 0.1
 * x / y / b        0 / 0.5 / 1
 * a                toggle sweep SWEEP_MIN..SWEEP_MAX
 * cr servos: left stick y = power
 */
@Config("ServoTester")
@TeleOp(name = "ServoTester", group = "Testers")
public class ServoTester extends LinearOpMode {
    public static double SWEEP_MIN = 0.2;
    public static double SWEEP_MAX = 0.8;
    public static double SWEEP_S = 1.5;

    private final List<String> names = new ArrayList<>();
    private final List<Object> servos = new ArrayList<>();
    private final List<String> notes = new ArrayList<>();

    @Override
    public void runOpMode() {
        for (String name : RobotConfig.HARDWARE.names(Kind.SERVO)) {
            String type = RobotConfig.HARDWARE.typeOf(name);
            Object s;
            if (Type.CR_SERVO.equals(type)) s = hardwareMap.tryGet(CRServo.class, name);
            else if (Type.SERVO.equals(type)) s = hardwareMap.tryGet(Servo.class, name);
            else {
                notes.add("skipping " + name + " (" + type + ")");
                continue;
            }
            if (s == null) {
                notes.add("missing on robot: " + name);
                continue;
            }
            names.add(name);
            servos.add(s);
        }

        Toggle up = new Toggle(), down = new Toggle(), left = new Toggle(), right = new Toggle();
        Toggle lb = new Toggle(), rb = new Toggle(), x = new Toggle(), y = new Toggle(), b = new Toggle(), a = new Toggle();
        double[] target = new double[servos.size()];
        java.util.Arrays.fill(target, Double.NaN);
        boolean sweeping = false;
        ElapsedTime sweep = new ElapsedTime();
        int sel = 0;

        telemetry.addLine(servos.size() + " servos, nothing moves until you press");
        for (String n : notes) telemetry.addLine(n);
        telemetry.update();
        waitForStart();
        if (servos.isEmpty()) return;

        while (opModeIsActive()) {
            int prev = sel;
            if (up.momentary(gamepad1.dpad_up)) sel--;
            if (down.momentary(gamepad1.dpad_down)) sel++;
            sel = TesterUtil.wrapIndex(sel, servos.size());
            if (sel != prev) {
                sweeping = false;
                if (servos.get(prev) instanceof CRServo) ((CRServo) servos.get(prev)).setPower(0);
            }
            Object s = servos.get(sel);
            String name = names.get(sel);

            telemetry.addData("servo", "%s  (%d/%d)", name, sel + 1, servos.size());
            telemetry.addData("port", RobotConfig.HARDWARE.whereIs(name));
            if (s instanceof CRServo) {
                double power = -gamepad1.left_stick_y;
                ((CRServo) s).setPower(power);
                telemetry.addData("power", "%.2f", power);
            } else {
                double t = target[sel], start = Double.isNaN(t) ? 0.5 : t;
                if (left.momentary(gamepad1.dpad_left)) t = start - 0.01;
                if (right.momentary(gamepad1.dpad_right)) t = start + 0.01;
                if (lb.momentary(gamepad1.left_bumper)) t = start - 0.1;
                if (rb.momentary(gamepad1.right_bumper)) t = start + 0.1;
                if (x.momentary(gamepad1.x)) t = 0;
                if (y.momentary(gamepad1.y)) t = 0.5;
                if (b.momentary(gamepad1.b)) t = 1;
                if (a.momentary(gamepad1.a)) {
                    sweeping = !sweeping;
                    sweep.reset();
                }
                if (sweeping) t = ((int) (sweep.seconds() / SWEEP_S)) % 2 == 0 ? SWEEP_MIN : SWEEP_MAX;
                if (!Double.isNaN(t)) {
                    t = Range.clip(t, 0, 1);
                    target[sel] = t;
                    ((Servo) s).setPosition(t);
                }
                telemetry.addData("position", Double.isNaN(t) ? "not commanded" : String.format("%.2f", t));
                if (sweeping) telemetry.addLine("sweeping");
            }
            telemetry.addLine();
            for (int i = 0; i < names.size(); i++) telemetry.addLine((i == sel ? "> " : "   ") + names.get(i));
            telemetry.update();
        }
    }
}
