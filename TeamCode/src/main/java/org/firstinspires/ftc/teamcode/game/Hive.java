package org.firstinspires.ftc.teamcode.game;

public final class Hive {
    public enum Cell {
        LOW, HIGH;

        public Cell other() {
            return this == LOW ? HIGH : LOW;
        }
    }

    /** last auto state, teleop takes it once */
    public static Cell autoEndUp;

    private final Alliance alliance;
    private Cell up;
    private int tips;

    public Hive(Alliance alliance) {
        this(alliance, Field.startUp(alliance));
    }

    public Hive(Alliance alliance, Cell up) {
        this.alliance = alliance;
        this.up = up;
    }

    public void tipped() {
        up = up.other();
        tips++;
    }

    public Cell up() {
        return up;
    }

    public int tips() {
        return tips;
    }

    public double targetX() {
        return Field.hiveX(alliance);
    }

    public double targetY() {
        return Field.cellY(up);
    }
}
