package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;

import java.lang.reflect.Field;

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

    /// shooter units (ShooterConfig.TICKS_PER_REV, GEAR_RATIO)

    public static double motorTps(double flywheelRpm) {
        return flywheelRpm / ShooterConfig.GEAR_RATIO * ShooterConfig.TICKS_PER_REV / 60.0;
    }

    public static double flywheelRpm(double motorTps) {
        return motorTps * 60.0 / ShooterConfig.TICKS_PER_REV * ShooterConfig.GEAR_RATIO;
    }
}
