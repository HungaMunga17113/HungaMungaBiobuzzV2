package org.firstinspires.ftc.teamcode.game;

public final class Hive {
    public enum Cell {
        LOW, HIGH;

        public Cell other() {
            return this == LOW ? HIGH : LOW;
        }
    }

    private final Alliance alliance;
    private Cell up;
    private int tips;

    public Hive(Alliance alliance) {
        this.alliance = alliance;
        this.up = Field.startUp(alliance);
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
