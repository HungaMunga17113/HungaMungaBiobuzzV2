package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig.Type.*;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.util.Conversions;
import org.firstinspires.ftc.teamcode.util.drivetrain.Curve;
import org.firstinspires.ftc.teamcode.util.drivetrain.CurveParams;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;

@Config
public class RobotConfig {
    public static boolean COMPETITION = false; // missing throw, rumbles, auto -> teleop carryover
    public static boolean STATIC_AIM = false; // turret locked forward, flywheel at ShooterConfig.targetRpm

    /**
     use pull/push config run configs
     */
    public static final HardwareConfig HARDWARE = new HardwareConfig("TimmehV2")
            .controlHub(hub -> hub
                    .motor(0, Names.BACK_LEFT, GOBILDA_5202)
                    .motor(1, Names.FRONT_LEFT, GOBILDA_5202)
                    .motor(2, Names.FRONT_RIGHT, GOBILDA_5202)
                    .motor(3, Names.BACK_RIGHT, GOBILDA_5202)
                    .crServo(1, Names.TURRET)
                    .servo(2, Names.FLOWER)
                    .servo(3, Names.TRANSFER)
                    .analog(0, Names.TURRET_ENCODER)
                    .digital(0, Names.INTAKE_BEAM))
            .expansionHub(2, hub -> hub
                    .motor(0, Names.INTAKE_2, GOBILDA_5202)
                    .motor(1, Names.FLYWHEEL[1], GOBILDA_5202)
                    .motor(2, Names.INTAKE, GOBILDA_5202)
                    .motor(3, Names.FLYWHEEL[0], GOBILDA_5202)
                    .i2c(1, Names.PINPOINT, GOBILDA_PINPOINT));

    public static final class Names {
        public static final String BACK_LEFT = "backLeft";
        public static final String FRONT_LEFT = "frontLeft";
        public static final String FRONT_RIGHT = "frontRight";
        public static final String BACK_RIGHT = "backRight";
        public static final String PINPOINT = "pinpoint";
        public static final String INTAKE = "intake";
        public static final String INTAKE_2 = "intake2";
        public static final String INTAKE_BEAM = "intakeBeam";
        public static final String TRANSFER = "transfer";
        public static final String FLOWER = "flower";
        public static final String TURRET = "turret";
        public static final String TURRET_ENCODER = "turretEncoder";
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
        public static Direction intake = FORWARD;
        public static Direction intake2 = FORWARD;

        public static Servo.Direction transfer = Servo.Direction.FORWARD;
        public static Servo.Direction flower = Servo.Direction.FORWARD;
        public static Servo.Direction turret = Servo.Direction.FORWARD;

        private MotorConfig() {}
    }

    @Config("Drivetrain")
    public static class DrivetrainConfig {
        public static boolean useFieldCentric = true;
        public static Curve DRIVE_CURVE = Curve.EXPONENTIAL; // linear, cubic_bezier, smoothstep, exponential, quintic, (ALL CAPS)
        public static CurveParams CURVE_PARAMS = new CurveParams();
        public static double SLOW_SCALE = 0.2;
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
        public static double kP = 0.0005, kI = 0, kD = 0;
        public static double kS = 0.05, kV = 1.0 / 6000, kA = 0; // untuned seed, run SVATuner + paste here

        public static double READY_TOLERANCE_RPM = 150; // atSpeed() band
        public static double READY_SETTLE_MS = 100; // in band this long

        /**
         * shoot from anywhere; {distance in, angle deg, rpm, tof s} per recorded spot
         * deg ccw, red frame, blue rotated 180
         */
        public static final double[][] SHOTS = {
                // {36, 90, 3200, 0.35},
                // {72, 90, 3900, 0.50},
                // {72, 45, 4000, 0.52},
        };

    }

    /** 0 = forward (intake side), ccw+ (unit circle, seen from above) */
    @Config("Turret")
    public static class TurretConfig {
        public static double MIN_DEG = -150;
        public static double MAX_DEG = 150;
        public static double OFFSET_FORWARD = 0; // turret pivot from robot center, in
        public static double OFFSET_LEFT = 0;

        public static double kP = 0.02, kI = 0, kD = 0.0005, kS = 0.04; // power per turret deg
        public static double kW = 0.0015; // power per chassis deg/s, untuned
        public static double TOLERANCE_DEG = 3; // settled band, kS off inside it

        // analog; rev / 0..3.3 v
        public static double ENCODER_ZERO_V = 1.65; // turret facing forward, read off telemetry
        public static double ENCODER_RATIO = 1; // turret rev per shaft rev, negative if it reads backwards
    }
    @Config("Intake")
    public static class IntakeConfig {
        public static double IN_POWER = 1;
        public static double OUT_POWER = -0.6;
        public static double FULL_POWER = 0; // stage 1 once full, negative = spit back out
        public static int MAX_BALLS = 4;
        public static boolean BEAM_CLEAR_HIGH = true;
    }
    @Config("Transfer")
    public static class TransferConfig {
        public static double REST_POS = 0;
        public static double FEED_POS = 0.5;
        public static double FEED_MS = 250; // time be4 return
        public static double RETURN_MS = 150; // back at rest be4 the next kick
    }
    @Config("Auto")
    public static class AutoConfig {
        public static int VOLLEY_BALLS = 4;
        public static int[] TIP_BALLS = {2, 8}; // per tip volley
        public static double TIP_PAUSE_MS = 1000;
        public static double PARK_AT_S = 28.1;
        public static double VOLLEY_TIMEOUT_MS = 3000;
        public static double INTAKE_DWELL_MS = 500;
        public static double FLOWER_MS = 600;
    }
    @Config("Flower")
    public static class FlowerConfig {
        public static double REST_POS = 0;
        public static double FEED_POS = 0.5;
    }
}
