package org.firstinspires.ftc.teamcode.util.control;

/** gains are tuned as power at NOMINAL volts */
public final class Voltage {
    public static final double NOMINAL = 12.4;
    private static final double MIN_SANE = 7;

    private Voltage() {}

    /** nominal power -> power to send at this battery voltage */
    public static double compensate(double nominalPower, double volts) {
        return clamp(nominalPower * NOMINAL / sane(volts));
    }

    /** power sent at this voltage -> equivalent power at NOMINAL */
    public static double toNominal(double power, double volts) {
        return power * sane(volts) / NOMINAL;
    }

    /** biggest nominal power the battery can deliver right now */
    public static double maxNominal(double volts) {
        return sane(volts) / NOMINAL;
    }

    private static double sane(double volts) {
        return Double.isNaN(volts) || volts < MIN_SANE ? NOMINAL : volts;
    }

    private static double clamp(double p) {
        return Math.max(-1, Math.min(1, p));
    }
}
