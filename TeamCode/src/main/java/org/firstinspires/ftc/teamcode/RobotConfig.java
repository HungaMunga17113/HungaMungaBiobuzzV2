package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Type.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
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

    /** device names, must match HARDWARE above */
    public static final class Names {
        public static final String BACK_LEFT = "backLeft";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_RIGHT = "backRight";
        public static final String PINPOINT = "pinpoint";
        /** add a name here (and a direction in MotorConfig) for more flywheel motors */
        public static final String[] FLYWHEEL = {"flywheel1", "flywheel2"};
        private Names() {}
    }

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
            return new Direction[]{flywheel1, flywheel2};
        }
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

        public static double motorTps(double flywheelRpm) {
            return flywheelRpm / GEAR_RATIO * TICKS_PER_REV / 60.0;
        }

        public static double flywheelRpm(double motorTps) {
            return motorTps * 60.0 / TICKS_PER_REV * GEAR_RATIO;
        }
    }

}
