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

    /** device names, checked against HARDWARE by drift() */
    public static final class Names {
        public static final String BACK_LEFT = "backLeft";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_RIGHT = "backRight";
        public static final String PINPOINT = "pinpoint";
        public static final String INTAKE = "intake";
        public static final String TRANSFER = "transfer";
        public static final String[] FLYWHEEL = {"flywheel1", "flywheel2"};
        private Names() {}
    }

    /** 1:1 names to HARDWARE */
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

        private MotorConfig() {}
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
        public static double TICKS_PER_REV = Conversions.TPR_6000_RPM;
        public static double GEAR_RATIO = 1; // flywheel rev / motor rev
        public static double targetRpm = 5000;
        public static boolean voltageComp = true;

        // ex controller: power at Voltage.NOMINAL, per flywheel rpm
        public static double kP = 0, kI = 0, kD = 0, kF = 0;
        public static double kS = 0, kV = 0, kA = 0;

        // sdk hub velocity pidf, hub units
        public static double sdkP = 5, sdkI = 3, sdkD = 0, sdkF = 12;

        public static double READY_TOLERANCE_RPM = 150; // atSpeed() band

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
        public static double FEED_MS = 250; // time be4 return
    }
}
