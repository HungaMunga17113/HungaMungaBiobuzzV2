package org.firstinspires.ftc.teamcode.teleop.testing;

import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;

final class TesterUtil {
    private TesterUtil() {}

    /** MotorConfig field with the device's name, or null */
    static Direction configuredDirection(String deviceName) {
        return MotorConfig.find(deviceName);
    }

    static int wrapIndex(int i, int size) {
        return size == 0 ? 0 : ((i % size) + size) % size;
    }

    static String sign(double v, double deadband) {
        return v > deadband ? "+" : v < -deadband ? "-" : "0";
    }
}
