package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConfig.MotorConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(Names.FRONT_LEFT);
        c.frontRightName.set(Names.FRONT_RIGHT);
        c.backLeftName.set(Names.BACK_LEFT);
        c.backRightName.set(Names.BACK_RIGHT);
        c.frontLeftDirection.set(MotorConfig.frontLeft);
        c.frontRightDirection.set(MotorConfig.frontRight);
        c.backLeftDirection.set(MotorConfig.backLeft);
        c.backRightDirection.set(MotorConfig.backRight);
    });

    // todo: offsets + directions from the pinpoint tuner
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set(Names.PINPOINT);
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.0);
        c.yPodOffset.set(0.0);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    // todo: v1's numbers as a start, rerun the foresight tuner on v2
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.0798)).put(2.5, Controller.proportional(0.216)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.0982)).put(2.5, Controller.proportional(0.266)));
        c.coast.set(Controller.proportionalFeedforward(0.0098));
        c.brake.set(Controller.proportionalFeedforward(0.0083));
        c.headingFeedback.set(Controller.proportional(15.7));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0812, 0.0481));
        c.linearBrakeCoefficients.set(Matrix.diag(0.0402, 0.0335));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.00262, 0.00240));
        c.maxAchievableForwardVelocity.set(88.8);
        c.maxAchievableStrafeVelocity.set(74.3);
        c.naturalForwardDeceleration.set(28.2);
        c.naturalStrafeDeceleration.set(54.9);
    });

    /** plain sdk follower, for the tuners */
    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig), new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }

    /** blaze follower, motor accel must already be engaged */
    public static Follower create(HardwareMap h, BlazePinpointLocalizer localizer) {
        return new Follower(localizer, new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig));
    }
}
