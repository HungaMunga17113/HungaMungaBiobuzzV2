package org.firstinspires.ftc.teamcode.util.control;

/** pid, d on measurement, output clamp, conditional anti-windup */
public class PIDControllerEx {
    protected double kP, kI, kD;
    private double minOutput = -1, maxOutput = 1;
    private double integral, prevMeasurement = Double.NaN;
    private long prevNanos;

    public PIDControllerEx(double kP, double kI, double kD) {
        setPID(kP, kI, kD);
    }

    public PIDControllerEx setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }

    public PIDControllerEx setOutputRange(double min, double max) {
        minOutput = min;
        maxOutput = max;
        return this;
    }

    public double calculate(double measurement, double setpoint) {
        long now = System.nanoTime();
        double dt = prevNanos == 0 ? 0 : (now - prevNanos) / 1e9;
        prevNanos = now;
        return calculate(measurement, setpoint, dt);
    }

    public double calculate(double measurement, double setpoint, double dt) {
        double error = setpoint - measurement;
        double derivative = dt > 0 && !Double.isNaN(prevMeasurement) ? -(measurement - prevMeasurement) / dt : 0;
        prevMeasurement = measurement;
        integral += error * dt;
        double raw = kP * error + kI * integral + kD * derivative + feedforward(setpoint, dt);
        double out = Math.max(minOutput, Math.min(raw, maxOutput));
        if (out != raw && Math.signum(error) == Math.signum(raw - out)) integral -= error * dt;
        return out;
    }

    protected double feedforward(double setpoint, double dt) {
        return 0;
    }

    public void reset() {
        integral = 0;
        prevMeasurement = Double.NaN;
        prevNanos = 0;
    }
}
