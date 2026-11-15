package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.anygeneric.blazeftc.BlazeDummyPlug;
import dev.anygeneric.blazeftc.PositionData;
import kotlin.Unit;

/** pedro localizer on blaze's pinpoint stream; mm -> config unit, setPose in software */
public class BlazePinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private final DistanceUnit unit;
    private boolean engaged;

    private volatile PoseOffset offset = PoseOffset.IDENTITY;
    private volatile double[] raw = {0, 0, 0};
    private volatile MotionState state = MotionState.ofVelocity(Pose.zero(), Velocity.zero());
    private volatile long lastNanos;
    private volatile int samples;

    public BlazePinpointLocalizer(HardwareMap hardwareMap, PinpointConfig config) {
        unit = config.globalDistanceUnit.get();
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, config.name.get());
        pinpoint.setOffsets(config.xPodOffset.get(), config.yPodOffset.get(), config.offsetUnits.get());
        if (config.ticksPerUnit.get().isPresent()) {
            pinpoint.setEncoderResolution(config.ticksPerUnit.get().getAsDouble(), config.encoderResolutionUnit.get());
        } else {
            pinpoint.setEncoderResolution(config.podType.get());
        }
        pinpoint.setEncoderDirections(config.xPodDirection.get(), config.yPodDirection.get());
        switch (config.resetMode.get()) {
            case RESET_AND_RECALIBRATE_IMU: pinpoint.resetPosAndIMU(); break;
            case RECALIBRATE_IMU: pinpoint.recalibrateIMU(); break;
            default: break;
        }
    }

    /** hand the pinpoint to blaze, once, before runBlazeFTC */
    public void engage() {
        if (engaged) throw new IllegalStateException("pinpoint already engaged");
        engaged = true;
        BlazeDummyPlug.engagePinpointAcceleration(pinpoint, d -> {
            accept(d);
            return Unit.INSTANCE;
        });
    }

    void accept(PositionData d) {
        accept(unit.fromMm(d.getXPosition()), unit.fromMm(d.getYPosition()), d.getDirection(),
                unit.fromMm(d.getXVelocity()), unit.fromMm(d.getYVelocity()), d.getAngVelocity());
    }

    /** raw sample in the config unit */
    void accept(double x, double y, double h, double vx, double vy, double omega) {
        raw = new double[]{x, y, h};
        PoseOffset o = offset;
        double[] p = o.apply(x, y, h);
        double[] v = o.rotate(vx, vy);
        state = MotionState.ofVelocity(new Pose(p[0], p[1], p[2]), new Velocity(v[0], v[1], omega));
        lastNanos = System.nanoTime();
        samples++;
    }

    @Override
    public void setPose(Pose pose) {
        double[] r = raw;
        offset = PoseOffset.aligning(r[0], r[1], r[2], pose.x(), pose.y(), pose.heading());
        state = state.withPose(new Pose(pose.x(), pose.y(), PoseOffset.wrap(pose.heading())));
    }

    @Override
    public MotionState state() {
        return state;
    }

    @Override
    public void update() {}

    @Override
    public void reset() {
        setPose(Pose.zero());
    }

    public boolean hasData() {
        return samples > 0;
    }

    public double msSinceData() {
        return samples == 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - lastNanos) / 1e6;
    }

    public int samples() {
        return samples;
    }

    public GoBildaPinpointDriver device() {
        return pinpoint;
    }
}
