package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfig;

/** stub: one motor at a fixed power. todo: ball counting / jam detection off current */
public final class Intake implements Subsystem {
    private final DcMotorEx motor;
    private double power;

    public Intake(DcMotorEx motor) {
        this.motor = motor;
    }

    public void set(double power) {
        if (power == this.power) return;
        this.power = power;
        motor.setPower(power);
    }

    public Command in() {
        return run(IntakeConfig.IN_POWER);
    }

    public Command out() {
        return run(IntakeConfig.OUT_POWER);
    }

    public Command stop() {
        return Commands.instant(() -> set(0)).requiring(this);
    }

    /** holds power until interrupted, then stops */
    private Command run(double p) {
        return Command.build()
                .setStart(() -> set(p))
                .setEnd(e -> set(0))
                .requiring(this);
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("intake", "%.2f", power);
    }
}
