package org.firstinspires.ftc.teamcode.game;

public enum Alliance {
    RED(0),
    BLUE(Math.PI);

    /** field heading the drivers look along, rad ccw+ */
    public final double driverForward;

    Alliance(double driverForward) {
        this.driverForward = driverForward;
    }

    /** last auto state, teleop takes it once */
    public static Alliance autoEnd;
}
