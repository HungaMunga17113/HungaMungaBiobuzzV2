package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.canvas.Canvas;

import java.util.ArrayDeque;
import java.util.Iterator;

/**
 * draws robot on the dashboard field
 * coords in inches, heading in radians (ccw+).
 *
 * usage:
 * <pre>
 * FieldDrawer field = new FieldDrawer();
 * TelemetryPacket packet = new TelemetryPacket();
 * field.trail(packet.fieldOverlay(), x, y)
 *      .robot(packet.fieldOverlay(), x, y, heading, "#3F51B5")
 *      .arrow(packet.fieldOverlay(), x, y, vx * 0.25, vy * 0.25, "#00BCD4");
 * dashboard.sendTelemetryPacket(packet);
 * </pre>
 */
public class FieldDrawer {
    public double robotSize = 18;
    public int trailPoints = 150;
    public String trailColor = "#4CAF5088";

    private final ArrayDeque<double[]> trail = new ArrayDeque<>();

    /** square robot with a line from the center to the front */
    public FieldDrawer robot(Canvas c, double x, double y, double heading, String color) {
        double half = robotSize / 2, cos = Math.cos(heading), sin = Math.sin(heading);
        double[] cx = {half, -half, -half, half}, cy = {half, half, -half, -half};
        double[] xs = new double[4], ys = new double[4];
        for (int i = 0; i < 4; i++) {
            xs[i] = x + cx[i] * cos - cy[i] * sin;
            ys[i] = y + cx[i] * sin + cy[i] * cos;
        }
        c.setStrokeWidth(2).setStroke(color).strokePolygon(xs, ys);
        c.strokeLine(x, y, x + half * cos, y + half * sin);
        return this;
    }

    /** adds (x, y) to the trail and draws it */
    public FieldDrawer trail(Canvas c, double x, double y) {
        trail.addLast(new double[]{x, y});
        while (trail.size() > Math.max(trailPoints, 2)) trail.removeFirst();
        if (trail.size() < 2) return this;
        double[] xs = new double[trail.size()], ys = new double[trail.size()];
        Iterator<double[]> it = trail.iterator();
        for (int i = 0; it.hasNext(); i++) {
            double[] p = it.next();
            xs[i] = p[0];
            ys[i] = p[1];
        }
        c.setStrokeWidth(1).setStroke(trailColor).strokePolyline(xs, ys);
        return this;
    }

    /** line from (x, y) by (dx, dy), in field inches */
    public FieldDrawer arrow(Canvas c, double x, double y, double dx, double dy, String color) {
        c.setStrokeWidth(2).setStroke(color).strokeLine(x, y, x + dx, y + dy);
        return this;
    }

    /** a robot-frame vector (forward, left), rotated into the field by heading */
    public FieldDrawer robotArrow(Canvas c, double x, double y, double heading, double forward, double left, String color) {
        double cos = Math.cos(heading), sin = Math.sin(heading);
        return arrow(c, x, y, forward * cos - left * sin, forward * sin + left * cos, color);
    }

    public void clearTrail() {
        trail.clear();
    }
}
