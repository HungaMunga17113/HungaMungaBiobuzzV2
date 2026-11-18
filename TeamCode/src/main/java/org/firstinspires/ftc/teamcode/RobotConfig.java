package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Type.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Config
public class RobotConfig {

    /**
     use pull/push config to manage this; remember to activate on the DS
     */
    public static final HardwareConfig HARDWARE = new HardwareConfig("TimmehV2")
            .controlHub(hub -> hub
                    .motor(0, "backLeft", GOBILDA_5202)
                    .motor(1, "frontLeft", GOBILDA_5202)
                    .motor(2, "frontRight", GOBILDA_5202)
                    .motor(3, "backRight", GOBILDA_5202)
                    .servo(3, "transfer"))
            .expansionHub(2, hub -> hub
                    .motor(2, "intake", GOBILDA_5202)
                    .motor(1, "flywheel2", GOBILDA_5202)
                    .motor(3, "flywheel1", GOBILDA_5202)
                    .i2c(1, "pinpoint", GOBILDA_PINPOINT));

    /** device names, checked against HARDWARE by drift() */
    public static final class Names {
        public static final String BACK_LEFT = "backLeft";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_RIGHT = "backRight";
        public static final String PINPOINT = "pinpoint";
        public static final String INTAKE = "intake";
        public static final String TRANSFER = "transfer";
        /** add a name here (and a direction in MotorConfig) for more flywheel motors */
        public static final String[] FLYWHEEL = {"flywheel1", "flywheel2"};
        private Names() {}
    }

    /** one Direction field per motor in HARDWARE, named exactly like the device */
    @Config("Motors")
    public static final class MotorConfig {
        public static Direction backLeft = FORWARD;
        public static Direction frontLeft = FORWARD;
        public static Direction backRight = REVERSE;
        public static Direction frontRight = REVERSE;
        public static Direction flywheel1 = REVERSE;
        public static Direction flywheel2 = FORWARD;
        // public static Direction transfer = FORWARD;
        public static Direction intake = FORWARD;

        /** same order as Names.FLYWHEEL */
        public static Direction[] flywheel() {
            return directions(Names.FLYWHEEL);
        }

        /** the field named deviceName, throws if there isn't one */
        public static Direction direction(String deviceName) {
            Direction d = find(deviceName);
            if (d == null) throw new IllegalArgumentException("no MotorConfig." + deviceName + " (add a Direction field named after the motor)");
            return d;
        }

        public static Direction[] directions(String... deviceNames) {
            Direction[] out = new Direction[deviceNames.length];
            for (int i = 0; i < deviceNames.length; i++) out[i] = direction(deviceNames[i]);
            return out;
        }

        /** the field named deviceName, or null */
        public static Direction find(String deviceName) {
            try {
                Field f = MotorConfig.class.getField(deviceName);
                return f.getType() == Direction.class ? (Direction) f.get(null) : null;
            } catch (ReflectiveOperationException e) {
                return null;
            }
        }

        private MotorConfig() {}
    }

    /** where Names / MotorConfig disagree with HARDWARE, empty when in sync */
    public static List<String> drift() {
        List<String> out = new ArrayList<>();
        try {
            for (Field f : Names.class.getFields()) {
                if (!Modifier.isStatic(f.getModifiers())) continue;
                Object v = f.get(null);
                String[] names = v instanceof String[] ? (String[]) v : v instanceof String ? new String[]{(String) v} : new String[0];
                for (String n : names) {
                    if (HARDWARE.typeOf(n) == null) out.add("Names." + f.getName() + " = \"" + n + "\" isn't in HARDWARE");
                }
            }
        } catch (IllegalAccessException e) {
            throw new IllegalStateException(e);
        }
        List<String> motors = HARDWARE.names(HardwareConfig.Kind.MOTOR);
        for (String m : motors) {
            if (MotorConfig.find(m) == null) out.add("motor \"" + m + "\" has no MotorConfig." + m + " direction");
        }
        for (Field f : MotorConfig.class.getFields()) {
            if (Modifier.isStatic(f.getModifiers()) && f.getType() == Direction.class && !motors.contains(f.getName())) {
                out.add("MotorConfig." + f.getName() + " isn't a motor in HARDWARE");
            }
        }
        return out;
    }

    @Config("Drivetrain")
    public static class DrivetrainConfig {
        public static boolean useFieldCentric = true;
        public static Curve DRIVE_CURVE = Curve.EXPONENTIAL; // linear, cubic_bezier, smoothstep, exponential, quintic, (ALL CAPS)
        public static CurveParams CURVE_PARAMS = new CurveParams();
        public static double AIM_TURN_SCALE = 0.2;
        public static double STICK_DB = 0.03;
        public static double TURN_SPEED = 0.8; // max rot power (after the curve)
    }

    @Config("Shooter")
    public static class ShooterConfig {
        public static double TICKS_PER_REV = Conversions.TPR_6000_RPM; // per flywheel motor encoder
        public static double GEAR_RATIO = 1; // flywheel rev per motor rev
        public static double targetRpm = 5000;
        public static boolean voltageComp = true;

        // ex controller: power at Voltage.NOMINAL, per flywheel rpm
        public static double kP = 0, kI = 0, kD = 0, kF = 0;
        public static double kS = 0, kV = 0, kA = 0;

        // sdk hub velocity pidf, hub units (defaults are the hub's)
        public static double sdkP = 10, sdkI = 3, sdkD = 0, sdkF = 0;

        public static double READY_TOLERANCE_RPM = 150; // atSpeed() band

        public static double motorTps(double flywheelRpm) {
            return flywheelRpm / GEAR_RATIO * TICKS_PER_REV / 60.0;
        }

        public static double flywheelRpm(double motorTps) {
            return motorTps * 60.0 / TICKS_PER_REV * GEAR_RATIO;
        }
    }

    /** 0 = forward (intake side), ccw+ (unit circle, seen from above) */
    @Config("Turret")
    public static class TurretConfig {
        public static double MIN_DEG = -170;
        public static double MAX_DEG = 170;
        public static double OFFSET_FORWARD = 0; // turret pivot from robot center, in
        public static double OFFSET_LEFT = 0;
        public static double LEAD_S = 0; // shoot-on-the-move lead (~ball flight time), 0 = off
    }

    @Config("Intake")
    public static class IntakeConfig {
        public static double IN_POWER = 1;
        public static double OUT_POWER = -0.6;
    }

    @Config("Transfer")
    public static class TransferConfig {
        public static double REST_POS = 0;
        public static double FEED_POS = 0.5;
        public static double FEED_MS = 250; // time at FEED_POS be4 returning
    }
}
