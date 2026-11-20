package org.firstinspires.ftc.teamcode.game;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;

/**
 * pedro field; origin bottom-left, +x right, +y up (top), inches, heading ccw+ rad
 *
 * numbers are roughly +-1 in; TODO: fix all measurements
 */
@Config("Field")
public final class Field {
    public static double SIZE = 144;

    /** hive pivots: each hive sits on its own drivers' side, red drivers at x = 0 */
    public static double RED_HIVE_X = 58.7;
    public static double BLUE_HIVE_X = 85.3;
    /** each hive tips along y, so its two cell openings sit at these y's (manual: ~18.8 in apart at the pivot) */
    public static double CELL_LOW_Y = 59;
    public static double CELL_HIGH_Y = 85;

    /** which cell is up at match start (the one the nectar is staged in) */
    public static Hive.Cell RED_START_UP = Hive.Cell.LOW;
    public static Hive.Cell BLUE_START_UP = Hive.Cell.HIGH;

    public static double ROBOT_LENGTH = 18; // f2b in inches
    public static double ROBOT_WIDTH = 18;

    private Field() {}

    public static double hiveX(Alliance a) {
        return a == Alliance.RED ? RED_HIVE_X : BLUE_HIVE_X;
    }

    public static double cellY(Hive.Cell c) {
        return c == Hive.Cell.LOW ? CELL_LOW_Y : CELL_HIGH_Y;
    }

    public static Hive.Cell startUp(Alliance a) {
        return a == Alliance.RED ? RED_START_UP : BLUE_START_UP;
    }

    /**
     * corners w/o gardens + facing away from drivers
     */
    public static Pose corner(Alliance a) {
        double cx = ROBOT_LENGTH / 2, cy = ROBOT_WIDTH / 2;
        return a == Alliance.RED
                ? new Pose(cx, SIZE - cy, 0)
                : new Pose(SIZE - cx, cy, Math.PI);
    }
}
