package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.IntakeConfig;

/** stage 1 + stage 2, beam break between them counts balls. null beam = no counting */
public final class Intake implements Subsystem {
    private final DcMotorEx stage1, stage2;
    private final DigitalChannel beam;
    private double power, sent1 = Double.NaN, sent2 = Double.NaN;
    private boolean broken;
    private int passed;

    public Intake(DcMotorEx stage1, DcMotorEx stage2, DigitalChannel beam) {
        this.stage1 = stage1;
        this.stage2 = stage2;
        this.beam = beam;
    }

    public void set(double power) {
        this.power = power;
    }

    public int balls() {
        return passed + (broken ? 1 : 0);
    }

    public boolean full() {
        return beam != null && balls() >= IntakeConfig.MAX_BALLS;
    }

    /** call once per ball fed to the shooter */
    public void fired() {
        passed = Math.max(0, passed - 1);
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

    public Command shot() {
        return Commands.instant(this::fired);
    }

    private Command run(double p) {
        return Command.build()
                .setStart(() -> set(p))
                .setEnd(e -> set(0))
                .requiring(this);
    }

    @Override
    public void periodic() {
        if (beam != null) {
            boolean now = beam.getState() != IntakeConfig.BEAM_CLEAR_HIGH;
            if (now && !broken && power < 0) passed = Math.max(0, passed - 1);
            if (!now && broken && power >= 0) passed++;
            broken = now;
        }
        double p1 = power > 0 && full() ? IntakeConfig.FULL_POWER : power;
        if (p1 != sent1) stage1.setPower(sent1 = p1);
        if (power != sent2) stage2.setPower(sent2 = power);
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("intake", "%.2f  %d balls%s", power, balls(), full() ? " FULL" : "");
    }
}
