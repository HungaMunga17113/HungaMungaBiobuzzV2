package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.FlowerConfig;

/** two-position servo: REST_POS / FEED_POS, holds whichever it was last set to */
public final class Flower implements Subsystem {
    private final Servo servo;
    private double position = Double.NaN;
    private boolean feeding;

    public Flower(Servo servo) {
        this.servo = servo;
    }

    public void set(double pos) {
        if (pos == position) return;
        position = pos;
        servo.setPosition(pos);
    }

    public void feed(boolean on) {
        feeding = on;
        set(on ? FlowerConfig.FEED_POS : FlowerConfig.REST_POS);
    }

    public void toggle() {
        feed(!feeding);
    }

    public boolean isFeeding() {
        return feeding;
    }

    // ivy commands, for auto

    public Command rest() {
        return instant(() -> feed(false)).requiring(this);
    }

    public Command feed() {
        return instant(() -> feed(true)).requiring(this);
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("flower", "%s  (%.2f)", feeding ? "FEED" : "rest", position);
    }
}
