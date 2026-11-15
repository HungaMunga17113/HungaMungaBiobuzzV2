package org.firstinspires.ftc.teamcode.util.hardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.junit.Before;
import org.junit.Test;

import java.util.Collections;

public class HwTest {
    private HardwareMap map;
    private DcMotorEx intake;
    private Hw hw;

    @Before
    public void setUp() {
        map = mock(HardwareMap.class);
        intake = mock(DcMotorEx.class);
        when(map.tryGet(DcMotorEx.class, "intake")).thenReturn(intake);
        hw = new Hw(map);
    }

    @Test
    public void getReturnsPresentDevices() {
        assertSame(intake, hw.get(DcMotorEx.class, "intake"));
        assertTrue(hw.missing().isEmpty());
    }

    @Test
    public void getNamesTheMissingDevice() {
        try {
            hw.get(DcMotorEx.class, "flywheel");
            fail();
        } catch (IllegalArgumentException e) {
            assertTrue(e.getMessage().contains("\"flywheel\""));
        }
    }

    @Test
    public void optionalRecordsMissingOnce() {
        assertNull(hw.optional(DcMotorEx.class, "flywheel"));
        assertNull(hw.optional(DcMotorEx.class, "flywheel"));
        assertEquals(Collections.singletonList("flywheel"), hw.missing());
    }

    @Test
    public void stubSwallowsCallsAndReturnsDefaults() {
        DcMotorEx m = hw.orStub(DcMotorEx.class, "flywheel");
        assertTrue(Hw.isStub(m));
        m.setPower(1);
        m.setDirection(DcMotorSimple.Direction.REVERSE);
        assertEquals(0.0, m.getPower(), 0);
        assertEquals(0, m.getCurrentPosition());
        assertFalse(m.isBusy());
        assertEquals(DcMotorSimple.Direction.FORWARD, m.getDirection());
        assertEquals(m, m);
    }

    @Test
    public void presentDevicesAreNotStubs() {
        assertFalse(Hw.isStub(hw.orStub(DcMotorEx.class, "intake")));
        assertFalse(Hw.isStub(null));
    }

    @Test(expected = IllegalArgumentException.class)
    public void stubNeedsAnInterface() {
        Hw.stub(Object.class);
    }

    @Test
    public void stubWorksForOtherInterfaces() {
        assertEquals(0.0, Hw.stub(VoltageSensor.class).getVoltage(), 0);
    }
}
