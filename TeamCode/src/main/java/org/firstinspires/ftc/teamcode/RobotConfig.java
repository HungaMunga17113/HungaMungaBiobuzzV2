package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Type.*;

import com.acmerobotics.dashboard.config.Config;

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
                    .motor(3, "flywheel", GOBILDA_5202)
                    .i2c(1, "pinpoint", GOBILDA_PINPOINT));

    /** device names, must match HARDWARE above */
    public static final class Names {
        public static final String BACK_LEFT = "backLeft";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_RIGHT = "backRight";
        public static final String PINPOINT = "pinpoint";
        private Names() {}
    }

    @Config("Motors")
    public static final class MotorConfig {
        public static Direction backLeft = FORWARD;
        public static Direction frontLeft = FORWARD;
        public static Direction backRight = REVERSE;
        public static Direction frontRight = REVERSE;
        public static Direction shooter = REVERSE;
        // public static Direction transfer = FORWARD;
        public static Direction intake = FORWARD;

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

    public static class ShooterConfig {
        public static double kP = 0;
        public static double kI = 0;
        public static double kD = 0;
        public static double kF = 0;
        public static double targetVel = 5000;
    }

}
