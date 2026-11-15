package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/** motors on one shaft driven as one, velocity is the mean of all encoders */
public final class MotorGroup {
    private final DcMotorEx[] motors;

    public MotorGroup(DcMotorEx... motors) {
        if (motors.length == 0) throw new IllegalArgumentException("empty motor group");
        this.motors = motors.clone();
    }

    /** names[i] gets dirs[i] */
    public static MotorGroup of(HardwareMap map, String[] names, Direction[] dirs) {
        if (names.length != dirs.length) throw new IllegalArgumentException(names.length + " names but " + dirs.length + " directions");
        DcMotorEx[] m = new DcMotorEx[names.length];
        for (int i = 0; i < names.length; i++) {
            m[i] = map.get(DcMotorEx.class, names[i]);
            m[i].setDirection(dirs[i]);
        }
        return new MotorGroup(m);
    }

    public int size() {
        return motors.length;
    }

    public DcMotorEx get(int i) {
        return motors[i];
    }

    public void setPower(double power) {
        for (DcMotorEx m : motors) m.setPower(power);
    }

    public void setVelocity(double tps) {
        for (DcMotorEx m : motors) m.setVelocity(tps);
    }

    public void setMode(DcMotor.RunMode mode) {
        for (DcMotorEx m : motors) m.setMode(mode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior b) {
        for (DcMotorEx m : motors) m.setZeroPowerBehavior(b);
    }

    public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {
        for (DcMotorEx m : motors) m.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public double getVelocity() {
        double sum = 0;
        for (DcMotorEx m : motors) sum += m.getVelocity();
        return sum / motors.length;
    }

    public double getPower() {
        double sum = 0;
        for (DcMotorEx m : motors) sum += m.getPower();
        return sum / motors.length;
    }

    /** total draw */
    public double getCurrent(CurrentUnit unit) {
        double sum = 0;
        for (DcMotorEx m : motors) sum += m.getCurrent(unit);
        return sum;
    }

    /** max minus min encoder velocity, big means a dead encoder or a flipped direction */
    public double velocitySpread() {
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (DcMotorEx m : motors) {
            double v = m.getVelocity();
            lo = Math.min(lo, v);
            hi = Math.max(hi, v);
        }
        return hi - lo;
    }
}
