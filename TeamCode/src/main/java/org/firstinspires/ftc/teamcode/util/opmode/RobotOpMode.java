package org.firstinspires.ftc.teamcode.util.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hw;

import java.util.ArrayList;
import java.util.List;

/**
 * owns startup order, bulk caching + the pinpoint; get hardware in onInit only.
 * each loop: clear bulk cache -> update pinpoint (or follower, which updates it) -> onLoop
 */
@Config("RobotOpMode")
public abstract class RobotOpMode extends LinearOpMode {
    public static double TELEMETRY_MS = 50;

    protected final FtcDashboard dashboard = FtcDashboard.getInstance();
    private final List<DcMotorEx> motors = new ArrayList<>();
    private List<LynxModule> hubs;
    protected Hw hw;
    private PinpointLocalizer pinpoint;
    private GoBildaPinpointDriver pinpointDevice;
    private Follower follower;
    private VoltageSensor battery;
    private boolean inInit;

    private final ElapsedTime rateTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime telemetryTimer = new ElapsedTime();
    private int loops;
    protected double voltage = Double.NaN, loopHz;

    protected abstract void onInit();

    protected void onInitLoop() {}

    protected void onStart() {}

    protected abstract void onLoop();

    protected void onStop() {}

    /** throttled to TELEMETRY_MS */
    protected void onTelemetry(Telemetry ds, TelemetryPacket dash) {}

    @Override
    public final void runOpMode() {
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hardwareMap.voltageSensor.iterator().next();
        hw = new Hw(hardwareMap);

        inInit = true;
        onInit();
        inInit = false;

        while (opModeInInit()) {
            clearBulkCache();
            if (pinpoint != null) pinpoint.update();
            onInitLoop();
            for (String name : hw.missing()) telemetry.addLine("missing: " + name + " (stubbed)");
            if (pinpoint != null) telemetry.addData("pinpoint", pinpointDevice.getDeviceStatus());
            telemetry.update();
            sleep(50);
        }
        if (isStopRequested()) return;

        rateTimer.reset();
        voltageTimer.reset();
        telemetryTimer.reset();
        voltage = battery.getVoltage();
        onStart();
        try {
            while (opModeIsActive()) {
                clearBulkCache();
                if (follower != null) follower.update();
                else if (pinpoint != null) pinpoint.update();
                onLoop();
                stats();
            }
        } finally {
            if (follower != null) follower.stop();
            for (DcMotorEx m : motors) m.setPower(0);
            onStop();
        }
    }

    private void clearBulkCache() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
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

    /** updated by the base before onLoop */
    protected final PinpointLocalizer pinpoint() {
        if (pinpoint == null) {
            requireInit("pinpoint");
            pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
            pinpointDevice = hardwareMap.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get());
        }
        return pinpoint;
    }

    /** updated by the base before onLoop (this also updates the pinpoint) */
    protected final Follower follower() {
        if (follower == null) {
            requireInit("follower");
            follower = Constants.create(hardwareMap, pinpoint());
        }
        return follower;
    }

    /** true if there's no pinpoint or it isn't reporting READY (calibrating, pod fault, bad read) */
    protected final boolean pinpointStale() {
        return pinpointDevice == null || pinpointDevice.getDeviceStatus() != DeviceStatus.READY;
    }

    private void requireInit(String what) {
        if (!inInit) throw new IllegalStateException(what + "() has to be called from onInit");
    }

    private void stats() {
        loops++;
        if (rateTimer.seconds() >= 0.5) {
            loopHz = loops / rateTimer.seconds();
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
            packet.put("pinpoint status", pinpointDevice.getDeviceStatus().toString());
            packet.put("x (in)", p.x());
            packet.put("y (in)", p.y());
            packet.put("heading (deg)", Math.toDegrees(p.heading()));
        }
        telemetry.update();
        dashboard.sendTelemetryPacket(packet);
    }
}
