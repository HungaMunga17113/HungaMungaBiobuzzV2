package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConfig.TransferConfig;

public final class Transfer implements Subsystem {
    private final Servo servo;
    private double position = Double.NaN;
    private boolean feeding;
    private final ElapsedTime feedTimer = new ElapsedTime();

    public Transfer(Servo servo) {
        this.servo = servo;
    }

    public void set(double pos) {
        if (pos == position) return;
        position = pos;
        servo.setPosition(pos);
    }

    /** kicks one ball, periodic() brings it back to rest after FEED_MS */
    public void startFeed() {
        set(TransferConfig.FEED_POS);
        feeding = true;
        feedTimer.reset();
    }

    public boolean isFeeding() {
        return feeding;
    }

    @Override
    public void periodic() {
        if (feeding && feedTimer.milliseconds() >= TransferConfig.FEED_MS) {
            set(TransferConfig.REST_POS);
            feeding = false;
        }
    }

    // ivy commands, for auto

    public Command rest() {
        return instant(() -> set(TransferConfig.REST_POS)).requiring(this);
    }

    /** one ball */
    public Command feed() {
        return Command.build()
                .setStart(this::startFeed)
                .setDone(() -> !feeding)
                .requiring(this);
    }

    @Override
    public void telemetry(Telemetry t) {
        t.addData("transfer", "%.2f", position);
    }
}
