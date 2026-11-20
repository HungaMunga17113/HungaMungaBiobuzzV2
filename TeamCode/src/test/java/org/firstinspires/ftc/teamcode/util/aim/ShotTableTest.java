package org.firstinspires.ftc.teamcode.util.aim;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class ShotTableTest {
    // two angle bands, rows out of order on purpose
    private final ShotTable table = new ShotTable(new double[][]{
            {72, 90, 4000, 0.5},
            {36, 90, 3000, 0.3},
            {36, 0, 3200, 0.4},
            {72, 0, 4400, 0.6},
    });

    @Test
    public void interpolatesDistanceInsideABand() {
        assertEquals(3500, table.rpm(54, 90), 1e-9);
        assertEquals(0.4, table.tof(54, 90), 1e-9);
    }

    @Test
    public void blendsBetweenBands() {
        // halfway in angle: (3500 + 3800) / 2
        assertEquals(3650, table.rpm(54, 45), 1e-9);
    }

    @Test
    public void flatPastTheEdges() {
        assertEquals(4000, table.rpm(200, 170), 1e-9);
        assertEquals(3200, table.rpm(0, -30), 1e-9);
    }

    @Test
    public void oneAngleIsADistanceTable() {
        ShotTable t = new ShotTable(new double[][]{{36, 0, 3000, 0.3}, {72, 0, 4000, 0.5}});
        assertEquals(3500, t.rpm(54, 123), 1e-9);
    }

    @Test
    public void emptyFallsBack() {
        ShotTable t = new ShotTable(new double[0][]);
        assertTrue(Double.isNaN(t.rpm(50, 0)));
        assertEquals(0, t.tof(50, 0), 0);
    }

    @Test
    public void exactRowsAndMultiSegment() {
        ShotTable t = new ShotTable(new double[][]{{24, 0, 0, 0.3}, {72, 0, 0, 0.5}, {120, 0, 0, 0.9}});
        assertEquals(0.5, t.tof(72, 0), 1e-9);
        assertEquals(0.4, t.tof(48, 0), 1e-9);
        assertEquals(0.7, t.tof(96, 0), 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void duplicateDistanceThrows() {
        new ShotTable(new double[][]{{36, 0, 3000, 0.3}, {36, 0, 3100, 0.3}});
    }

    @Test(expected = IllegalArgumentException.class)
    public void shortRowThrows() {
        new ShotTable(new double[][]{{36, 0, 3000}});
    }
}
