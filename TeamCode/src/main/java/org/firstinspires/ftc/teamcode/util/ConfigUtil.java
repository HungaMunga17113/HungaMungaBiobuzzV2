package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

/** helpers that read RobotConfig, so RobotConfig itself stays values only */
public final class ConfigUtil {
    private ConfigUtil() {}

    /// motor directions

    /** MotorConfig.<deviceName>, throws if there isn't one */
    public static Direction direction(String deviceName) {
        Direction d = findDirection(deviceName);
        if (d == null) throw new IllegalArgumentException("no MotorConfig." + deviceName + " (add a Direction field named after the motor)");
        return d;
    }

    public static Direction[] directions(String... deviceNames) {
        Direction[] out = new Direction[deviceNames.length];
        for (int i = 0; i < deviceNames.length; i++) out[i] = direction(deviceNames[i]);
        return out;
    }

    public static Direction[] flywheelDirections() {
        return directions(Names.FLYWHEEL);
    }

    /** MotorConfig.<deviceName>, or null */
    public static Direction findDirection(String deviceName) {
        try {
            Field f = MotorConfig.class.getField(deviceName);
            return f.getType() == Direction.class ? (Direction) f.get(null) : null;
        } catch (ReflectiveOperationException e) {
            return null;
        }
    }

    /** MotorConfig.<deviceName> as a Servo.Direction, or null */
    public static Servo.Direction findServoDirection(String deviceName) {
        try {
            Field f = MotorConfig.class.getField(deviceName);
            return f.getType() == Servo.Direction.class ? (Servo.Direction) f.get(null) : null;
        } catch (ReflectiveOperationException e) {
            return null;
        }
    }

    /// drift

    /** where Names / MotorConfig disagree with HARDWARE, empty when in sync */
    public static List<String> drift() {
        List<String> out = new ArrayList<>();
        try {
            for (Field f : Names.class.getFields()) {
                if (!Modifier.isStatic(f.getModifiers())) continue;
                Object v = f.get(null);
                String[] names = v instanceof String[] ? (String[]) v : v instanceof String ? new String[]{(String) v} : new String[0];
                for (String n : names) {
                    if (RobotConfig.HARDWARE.typeOf(n) == null) out.add("Names." + f.getName() + " = \"" + n + "\" isn't in HARDWARE");
                }
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
        List<String> motors = RobotConfig.HARDWARE.names(HardwareConfig.Kind.MOTOR);
        for (String m : motors) {
            if (findDirection(m) == null) out.add("motor \"" + m + "\" has no MotorConfig." + m + " direction");
        }
        for (Field f : MotorConfig.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) && f.getType() == Direction.class && !motors.contains(f.getName())) {
                out.add("MotorConfig." + f.getName() + " isn't a motor in HARDWARE");
            }
        }
        List<String> servos = RobotConfig.HARDWARE.names(HardwareConfig.Kind.SERVO);
        for (Field f : MotorConfig.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) && f.getType() == Servo.Direction.class && !servos.contains(f.getName())) {
                out.add("MotorConfig." + f.getName() + " isn't a servo in HARDWARE");
            }
        }
        return out;
    }

    /// shooter units (ShooterConfig.TICKS_PER_REV, GEAR_RATIO)

    public static double motorTps(double flywheelRpm) {
        return flywheelRpm / ShooterConfig.GEAR_RATIO * ShooterConfig.TICKS_PER_REV / 60.0;
    }

    public static double flywheelRpm(double motorTps) {
        return motorTps * 60.0 / ShooterConfig.TICKS_PER_REV * ShooterConfig.GEAR_RATIO;
    }
}
