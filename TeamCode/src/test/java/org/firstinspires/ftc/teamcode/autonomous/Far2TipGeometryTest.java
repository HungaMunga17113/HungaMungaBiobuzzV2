package org.firstinspires.ftc.teamcode.autonomous;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.game.Alliance;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.util.aim.TurretAim;
import org.junit.Test;

public class Far2TipGeometryTest {
    private static final double EPS = 1e-9, MID = Field.SIZE / 2;

    private static Pose pose(Far2Tip auto, String name) throws Exception {
        java.lang.reflect.Field f = Far2Tip.class.getDeclaredField(name);
        f.setAccessible(true);
        return (Pose) f.get(auto);
    }

    @Test
    public void blueIsRedRotated180() throws Exception {
        Far2Tip red = new RedFar2Tip(), blue = new BlueFar2Tip();
        int n = 0;
        for (java.lang.reflect.Field f : Far2Tip.class.getDeclaredFields()) {
            if (f.getType() != Pose.class) continue;
            Pose r = pose(red, f.getName()), b = pose(blue, f.getName());
            assertEquals(f.getName(), 2 * MID - r.x(), b.x(), EPS);
            assertEquals(f.getName(), 2 * MID - r.y(), b.y(), EPS);
            assertEquals(f.getName(), 0, TurretAim.wrap(b.heading() - r.heading() - Math.PI), EPS);
            n++;
        }
        assertTrue(n > 0);
    }

    @Test
    public void keyPosesOnOwnSide() throws Exception {
        for (Far2Tip auto : new Far2Tip[]{new RedFar2Tip(), new BlueFar2Tip()}) {
            Alliance a = auto.alliance();
            for (Pose p : new Pose[]{Field.corner(a), pose(auto, "start"), pose(auto, "park")}) {
                assertTrue(a + " " + p, a == Alliance.RED ? p.x() < MID : p.x() > MID);
            }
        }
    }
}
