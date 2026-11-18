package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * teleop calls the plain methods directly; the ivy command factories (requiring(this)) are for auto.
 * call periodic() on every subsystem once per loop.
 */
public interface Subsystem {
    default void periodic() {}
    default void telemetry(Telemetry t) {}
}
