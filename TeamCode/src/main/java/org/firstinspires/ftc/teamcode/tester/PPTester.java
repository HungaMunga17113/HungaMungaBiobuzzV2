package org.firstinspires.ftc.teamcode.tester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.control.Toggle;

/**
 * checks Constants.localizerConfig by pushing the robot by hand.
 * back  zero pose + checks
 */
@TeleOp(name = "PPTester", group = "Testers")
public class PPTester extends LinearOpMode {
    private static final double PUSH_IN = 6;

    @Override
    public void runOpMode() {
        GoBildaPinpointDriver pp = hardwareMap.tryGet(GoBildaPinpointDriver.class, Names.PINPOINT);
        if (pp == null) {
            telemetry.addLine("no \"" + Names.PINPOINT + "\" in the active config");
            telemetry.update();
            waitForStart();
            return;
        }
        PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Toggle reset = new Toggle();
        double maxX = 0, minX = 0, maxY = 0, minY = 0, turned = 0, lastH = 0;

        telemetry.addLine("keep still, pinpoint is calibrating");
        telemetry.update();
        waitForStart();
        localizer.setPose(Pose.zero());

        while (opModeIsActive()) {
            localizer.update();
            Pose p = localizer.pose();
            if (reset.momentary(gamepad1.back)) {
                localizer.setPose(Pose.zero());
                maxX = minX = maxY = minY = turned = lastH = 0;
                continue;
            }
            maxX = Math.max(maxX, p.x());
            minX = Math.min(minX, p.x());
            maxY = Math.max(maxY, p.y());
            minY = Math.min(minY, p.y());
            double dh = p.heading() - lastH;
            if (dh > Math.PI) dh -= 2 * Math.PI;
            if (dh < -Math.PI) dh += 2 * Math.PI;
            turned += dh;
            lastH = p.heading();

            GoBildaPinpointDriver.DeviceStatus status = pp.getDeviceStatus();
            telemetry.addData("status", status + (status == GoBildaPinpointDriver.DeviceStatus.READY ? "" : "  !!"));
            telemetry.addData("rate", "%.0f Hz", pp.getFrequency());
            telemetry.addData("pose", "x %.2f  y %.2f in  h %.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("encoders", "x %d  y %d", pp.getEncoderX(), pp.getEncoderY());
            telemetry.addLine();
            telemetry.addLine("push forward " + (int) PUSH_IN + "+ in: " + check(maxX, minX, "x", "xPodDirection"));
            telemetry.addLine("push left " + (int) PUSH_IN + "+ in:    " + check(maxY, minY, "y", "yPodDirection"));
            telemetry.addLine("turn ccw: heading should go up (" + String.format("%.0f°", Math.toDegrees(turned)) + " so far)");
            if (Math.abs(turned) > 2 * Math.PI) {
                telemetry.addData("after a full spin in place", "drift %.1f in (should be ~0, else pod offsets are off)", Math.hypot(p.x(), p.y()));
            }
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("x (in)", p.x());
            packet.put("y (in)", p.y());
            packet.put("heading (deg)", Math.toDegrees(p.heading()));
            packet.put("encoder x", pp.getEncoderX());
            packet.put("encoder y", pp.getEncoderY());
            packet.put("pinpoint (Hz)", pp.getFrequency());
            dashboard.sendTelemetryPacket(packet);
        }
    }

    private static String check(double max, double min, String axis, String fix) {
        if (max >= PUSH_IN && -min < PUSH_IN / 2) return "ok (+" + axis + ")";
        if (-min >= PUSH_IN && max < PUSH_IN / 2) return "!! went -" + axis + ", flip " + fix;
        return "waiting";
    }
}
