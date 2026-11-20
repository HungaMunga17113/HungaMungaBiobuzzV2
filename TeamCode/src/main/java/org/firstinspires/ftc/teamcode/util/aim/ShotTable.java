package org.firstinspires.ftc.teamcode.util.aim;

import org.firstinspires.ftc.teamcode.RobotConfig.ShooterConfig;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * recorded shots {distance in, angle deg, rpm, tof s}, looked up by (distance, angle).
 * rows sharing an angle form a band, linear along distance, bands blended along angle.
 * one angle (eg all 0) = plain distance table. flat past the edges.
 */
public final class ShotTable {
    private static final int RPM = 2, TOF = 3;

    private final double[] angles;
    private final double[][][] bands; // [band][row] = {distance, angle, rpm, tof}, sorted by distance

    public ShotTable(double[][] rows) {
        TreeMap<Double, List<double[]>> byAngle = new TreeMap<>();
        for (int i = 0; i < rows.length; i++) {
            if (rows[i].length != 4) throw new IllegalArgumentException("shot " + i + " needs {distance, angle, rpm, tof}");
            byAngle.computeIfAbsent(rows[i][1], k -> new ArrayList<>()).add(rows[i]);
        }
        angles = new double[byAngle.size()];
        bands = new double[byAngle.size()][][];
        int b = 0;
        for (Map.Entry<Double, List<double[]>> e : byAngle.entrySet()) {
            double[][] band = e.getValue().toArray(new double[0][]);
            Arrays.sort(band, (p, q) -> Double.compare(p[0], q[0]));
            for (int i = 1; i < band.length; i++)
                if (band[i][0] == band[i - 1][0]) throw new IllegalArgumentException("two shots at " + band[i][0] + " in, " + e.getKey() + " deg");
            angles[b] = e.getKey();
            bands[b++] = band;
        }
    }

    private static ShotTable config;

    /** ShooterConfig.SHOTS, built once */
    public static ShotTable fromConfig() {
        if (config == null) config = new ShotTable(ShooterConfig.SHOTS);
        return config;
    }

    public boolean isEmpty() {
        return angles.length == 0;
    }

    /** NaN if empty */
    public double rpm(double distance, double angleDeg) {
        return isEmpty() ? Double.NaN : blend(RPM, distance, angleDeg);
    }

    /** 0 if empty (no lead) */
    public double tof(double distance, double angleDeg) {
        return isEmpty() ? 0 : blend(TOF, distance, angleDeg);
    }

    private double blend(int col, double distance, double angleDeg) {
        int n = angles.length;
        if (n == 1 || Double.isNaN(angleDeg) || angleDeg <= angles[0]) return along(bands[0], col, distance);
        if (angleDeg >= angles[n - 1]) return along(bands[n - 1], col, distance);
        int i = Arrays.binarySearch(angles, angleDeg);
        if (i >= 0) return along(bands[i], col, distance);
        int hi = -i - 1, lo = hi - 1;
        double t = (angleDeg - angles[lo]) / (angles[hi] - angles[lo]);
        return along(bands[lo], col, distance) * (1 - t) + along(bands[hi], col, distance) * t;
    }

    private static double along(double[][] band, int col, double d) {
        int n = band.length - 1;
        if (Double.isNaN(d) || d <= band[0][0]) return band[0][col];
        if (d >= band[n][0]) return band[n][col];
        int i = 1;
        while (band[i][0] < d) i++;
        double t = (d - band[i - 1][0]) / (band[i][0] - band[i - 1][0]);
        return band[i - 1][col] + t * (band[i][col] - band[i - 1][col]);
    }
}
