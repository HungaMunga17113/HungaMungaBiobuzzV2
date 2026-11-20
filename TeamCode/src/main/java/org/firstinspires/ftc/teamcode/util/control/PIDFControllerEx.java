package org.firstinspires.ftc.teamcode.util.control;

/** pid + kS sgn(sp) + kV sp + kA d(sp)/dt, for velocity mechs */
public class PIDFControllerEx extends PIDControllerEx {
    private double kS, kV, kA, prevSetpoint = Double.NaN;

    public PIDFControllerEx(double kP, double kI, double kD) {
        super(kP, kI, kD);
    }

    public PIDFControllerEx setSVA(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    @Override
    protected double feedforward(double setpoint, double dt) {
        double accel = dt > 0 && !Double.isNaN(prevSetpoint) ? (setpoint - prevSetpoint) / dt : 0;
        prevSetpoint = setpoint;
        return kS * Math.signum(setpoint) + kV * setpoint + kA * accel;
    }

    @Override
    public void reset() {
        super.reset();
        prevSetpoint = Double.NaN;
    }
}
