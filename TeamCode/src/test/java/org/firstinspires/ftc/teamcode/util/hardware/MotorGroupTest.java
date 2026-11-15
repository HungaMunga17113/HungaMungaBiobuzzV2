package org.firstinspires.ftc.teamcode.util.hardware;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.junit.Before;
import org.junit.Test;

public class MotorGroupTest {
    private DcMotorEx a, b;
    private MotorGroup group;

    @Before
    public void setUp() {
        a = mock(DcMotorEx.class);
        b = mock(DcMotorEx.class);
        group = new MotorGroup(a, b);
    }

    @Test
    public void powerAndVelocityGoToEveryMotor() {
        group.setPower(0.6);
        group.setVelocity(2000);
        group.setVelocityPIDFCoefficients(1, 2, 3, 4);
        for (DcMotorEx m : new DcMotorEx[]{a, b}) {
            verify(m).setPower(0.6);
            verify(m).setVelocity(2000);
            verify(m).setVelocityPIDFCoefficients(1, 2, 3, 4);
        }
    }

    @Test
    public void velocityIsTheMean() {
        when(a.getVelocity()).thenReturn(2000.0);
        when(b.getVelocity()).thenReturn(2100.0);
        assertEquals(2050, group.getVelocity(), 1e-9);
        assertEquals(100, group.velocitySpread(), 1e-9);
    }

    @Test
    public void ofAppliesDirectionsInOrder() {
        HardwareMap map = mock(HardwareMap.class);
        when(map.get(DcMotorEx.class, "l")).thenReturn(a);
        when(map.get(DcMotorEx.class, "r")).thenReturn(b);
        MotorGroup g = MotorGroup.of(map, new String[]{"l", "r"}, new Direction[]{Direction.REVERSE, Direction.FORWARD});
        assertEquals(2, g.size());
        verify(a).setDirection(Direction.REVERSE);
        verify(b).setDirection(Direction.FORWARD);
    }

    @Test(expected = IllegalArgumentException.class)
    public void mismatchedLengthsThrow() {
        MotorGroup.of(mock(HardwareMap.class), new String[]{"l", "r"}, new Direction[]{Direction.FORWARD});
    }
}
