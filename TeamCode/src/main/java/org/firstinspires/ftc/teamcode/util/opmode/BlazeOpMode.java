package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.BlazePinpointLocalizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hw;

import java.util.ArrayList;
import java.util.List;

import dev.anygeneric.blazeftc.DummyPlugOpMode;

/** owns blaze startup order + the pinpoint; get hardware in onInit only */
@Config("BlazeOpMode")
public abstract class BlazeOpMode extends DummyPlugOpMode {
    public static double TELEMETRY_MS = 50;
    public static double PINPOINT_STALE_MS = 250;

    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final List<DcMotorEx> motors = new ArrayList<>();
    protected Hw hw;
    private BlazePinpointLocalizer pinpoint;
    private Follower follower;
    private VoltageSensor battery;
    private boolean inInit;

    private final ElapsedTime rateTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private int loops, lastSamples;
    protected double voltage = Double.NaN, loopHz, pinpointHz;

    protected abstract void onInit();

    protected void onInitLoop() {}

    protected void onStart() {}

    protected abstract void onLoop();

    protected void onStop() {}

    /** throttled to TELEMETRY_MS */
    protected void onTelemetry(Telemetry ds, TelemetryPacket dash) {}

    @Override
    public final void runOpModeInBlaze() {
        initializeBlazeFTC();
        engageMotorAcceleration();
        battery = hardwareMap.voltageSensor.iterator().next();
        hw = new Hw(hardwareMap);

        inInit = true;
        onInit();
        inInit = false;
        if (pinpoint != null) pinpoint.engage();

        while (opModeInInit()) {
            onInitLoop();
            for (String name : hw.missing()) telemetry.addLine("missing: " + name + " (stubbed)");
            if (pinpoint != null) telemetry.addLine("keep still, pinpoint is calibrating");
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        runBlazeFTC(0);
        rateTimer.reset();
        voltageTimer.reset();
        telemetryTimer.reset();
        voltage = battery.getVoltage();
        onStart();
        try {
            while (opModeIsActive()) {
                onLoop();
                if (follower != null) follower.update();
                stats();
            }
        } finally {
            if (follower != null) follower.stop();
            for (DcMotorEx m : motors) m.setPower(0);
            onStop();
        }
    }

    protected final DcMotorEx motor(String name, Direction direction) {
        requireInit("motor");
        return setup(hw.get(DcMotorEx.class, name), direction);
    }

    /** no-op stub if the motor isn't on the robot yet */
    protected final DcMotorEx optionalMotor(String name, Direction direction) {
        requireInit("optionalMotor");
        DcMotorEx m = hw.orStub(DcMotorEx.class, name);
        return Hw.isStub(m) ? m : setup(m, direction);
    }

    private DcMotorEx setup(DcMotorEx m, Direction direction) {
        m.setDirection(direction);
        m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motors.add(m);
        return m;
    }

    protected final BlazePinpointLocalizer pinpoint() {
        if (pinpoint == null) {
            requireInit("pinpoint");
            pinpoint = new BlazePinpointLocalizer(hardwareMap, Constants.localizerConfig);
        }
        return pinpoint;
    }

    /** updated by the base after onLoop */
    protected final Follower follower() {
        if (follower == null) {
            requireInit("follower");
            follower = Constants.create(hardwareMap, pinpoint());
        }
        return follower;
    }

    protected final boolean pinpointStale() {
        return pinpoint == null || pinpoint.msSinceData() > PINPOINT_STALE_MS;
    }

    private void requireInit(String what) {
        if (!inInit) throw new IllegalStateException(what + "() has to be called from onInit");
    }

    private void stats() {
        loops++;
        if (rateTimer.seconds() >= 0.5) {
            int samples = pinpoint == null ? 0 : pinpoint.samples();
            loopHz = loops / rateTimer.seconds();
            pinpointHz = (samples - lastSamples) / rateTimer.seconds();
            lastSamples = samples;
            loops = 0;
            rateTimer.reset();
        }
        if (voltageTimer.seconds() >= 1) {
            voltage = battery.getVoltage();
            voltageTimer.reset();
        }
        if (telemetryTimer.milliseconds() < TELEMETRY_MS) return;
        telemetryTimer.reset();

        TelemetryPacket packet = new TelemetryPacket();
        onTelemetry(telemetry, packet);
        packet.put("loop (Hz)", loopHz);
        packet.put("battery (V)", voltage);
        if (pinpoint != null) {
            Pose p = pinpoint.pose();
            packet.put("pinpoint (Hz)", pinpointHz);
            packet.put("x (in)", p.x());
            packet.put("y (in)", p.y());
            packet.put("heading (deg)", Math.toDegrees(p.heading()));
        }
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}
