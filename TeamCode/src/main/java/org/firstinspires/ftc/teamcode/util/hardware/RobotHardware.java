package org.firstinspires.ftc.teamcode.util.hardware;

import org.firstinspires.ftc.teamcode.util.ConfigUtil;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.List;

/**
 * shared hardware setup, same idea as the sdk's RobotHardware sample.
 * <pre>
 * // teleop (OpMode)
 * init()       robot = new RobotHardware(hardwareMap); intake = robot.motor(Names.INTAKE);
 * init_loop()  robot.update(); robot.addInitWarnings(telemetry);
 * loop()       robot.update(); ...
 *
 * // auto (LinearOpMode)
 * RobotHardware robot = new RobotHardware(hardwareMap);
 * waitForStart();
 * while (opModeIsActive()) { robot.update(); ...; telemetry.update(); }
 * </pre>
 * update() has to run at the top of every loop: it clears the bulk cache (MANUAL mode) and updates the pinpoint.
 */
public final class RobotHardware {
    public final HardwareMap hardwareMap;
    public final Hw hw;
    private final List<LynxModule> hubs;
    private final VoltageSensor battery;
    private PinpointLocalizer pinpoint;
    private GoBildaPinpointDriver pinpointDevice;
    private Follower follower;

    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double voltage;

    public RobotHardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.hw = new Hw(hardwareMap);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hardwareMap.voltageSensor.iterator().next();
        voltage = battery.getVoltage();
    }

    /** top of every loop: fresh bulk reads, then the follower (or just the pinpoint) */
    public void update() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        if (follower != null) follower.update();
        else if (pinpoint != null) pinpoint.update();
        if (voltageTimer.seconds() >= 1) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }
    }

    /** battery volts, read once a second (a voltage read is a slow i2c-ish call) */
    public double voltage() {
        return voltage;
    }

    /** missing optional devices, for the init screen */
    public void addInitWarnings(Telemetry telemetry) {
        for (String name : hw.missing()) telemetry.addLine("missing: " + name + " (stubbed)");
        if (pinpointDevice != null) telemetry.addData("pinpoint", pinpointDevice.getDeviceStatus());
    }

    /** direction from MotorConfig.<name>, brake, no encoder pid */
    public DcMotorEx motor(String name) {
        return setup(hw.get(DcMotorEx.class, name), ConfigUtil.direction(name));
    }

    /** no-op stub if the motor isn't on the robot yet */
    public DcMotorEx optionalMotor(String name) {
        DcMotorEx m = hw.orStub(DcMotorEx.class, name);
        return Hw.isStub(m) ? m : setup(m, ConfigUtil.direction(name));
    }

    /** no-op stub if the servo isn't on the robot yet, direction from MotorConfig.<name> (FORWARD if none) */
    public Servo optionalServo(String name) {
        Servo s = hw.orStub(Servo.class, name);
        if (Hw.isStub(s)) return s;
        Servo.Direction d = ConfigUtil.findServoDirection(name);
        s.setDirection(d == null ? Servo.Direction.FORWARD : d);
        return s;
    }

    /** no-op stub if missing, direction from the MotorConfig.<name> servo direction (FORWARD if none) */
    public CRServo optionalCrServo(String name) {
        CRServo s = hw.orStub(CRServo.class, name);
        if (Hw.isStub(s)) return s;
        Servo.Direction d = ConfigUtil.findServoDirection(name);
        s.setDirection(d == Servo.Direction.REVERSE ? Direction.REVERSE : Direction.FORWARD);
        return s;
    }

    /** null if it isn't on the robot yet */
    public AnalogInput optionalAnalog(String name) {
        return hw.optional(AnalogInput.class, name);
    }

    /** null if it isn't on the robot yet */
    public DigitalChannel optionalDigital(String name) {
        DigitalChannel d = hw.optional(DigitalChannel.class, name);
        if (d != null) d.setMode(DigitalChannel.Mode.INPUT);
        return d;
    }

    /** stage 1, stage 2, beam break */
    public Intake intake() {
        return new Intake(optionalMotor(Names.INTAKE), optionalMotor(Names.INTAKE_2), optionalDigital(Names.INTAKE_BEAM));
    }

    private static DcMotorEx setup(DcMotorEx m, Direction direction) {
        m.setDirection(direction);
        m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        return m;
    }

    public PinpointLocalizer pinpoint() {
        if (pinpoint == null) {
            pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
            pinpointDevice = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        }
        return pinpoint;
    }

    /** owns the drive motors, update() updates it */
    public Follower follower() {
        if (follower == null) follower = Constants.create(hardwareMap, pinpoint());
        return follower;
    }

    /** true if there's no pinpoint or it isn't READY (calibrating, pod fault, bad read) */
    public boolean pinpointStale() {
        return pinpointDevice == null || pinpointDevice.getDeviceStatus() != DeviceStatus.READY;
    }
}
