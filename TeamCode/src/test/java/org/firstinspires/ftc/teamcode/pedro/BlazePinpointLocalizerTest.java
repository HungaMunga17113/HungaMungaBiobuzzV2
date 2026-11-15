package org.firstinspires.ftc.teamcode.pedro;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.junit.Before;
import org.junit.Test;

import dev.anygeneric.blazeftc.PositionData;

public class BlazePinpointLocalizerTest {
    private static final double EPS = 1e-6;
    private GoBildaPinpointDriver device;
    private BlazePinpointLocalizer localizer;

    @Before
    public void setUp() {
        HardwareMap map = mock(HardwareMap.class);
        device = mock(GoBildaPinpointDriver.class);
        when(map.get(GoBildaPinpointDriver.class, Constants.localizerConfig.name.get())).thenReturn(device);
        localizer = new BlazePinpointLocalizer(map, Constants.localizerConfig);
    }

    @Test
    public void configuresTheDeviceLikePedrosLocalizer() {
        verify(device).setOffsets(Constants.localizerConfig.xPodOffset.get(), Constants.localizerConfig.yPodOffset.get(), DistanceUnit.INCH);
        verify(device).setEncoderDirections(Constants.localizerConfig.xPodDirection.get(), Constants.localizerConfig.yPodDirection.get());
    }

    @Test
    public void convertsBlazeMillimetersToInches() {
        PositionData d = new PositionData();
        d.setXPosition(254f);
        d.setYPosition(-25.4f);
        d.setDirection(0.5f);
        d.setXVelocity(508f);
        localizer.accept(d);
        assertEquals(10, localizer.pose().x(), 1e-4);
        assertEquals(-1, localizer.pose().y(), 1e-4);
        assertEquals(0.5, localizer.pose().heading(), 1e-6);
        assertEquals(20, localizer.velocity().vx, 1e-4);
    }

    @Test
    public void setPoseSticksAcrossNewSamples() {
        localizer.accept(5, 5, 1, 0, 0, 0);
        localizer.setPose(new Pose(72, 72, 0));
        assertEquals(72, localizer.pose().x(), EPS);
        localizer.accept(5, 5, 1, 0, 0, 0);
        assertEquals(72, localizer.pose().x(), EPS);
        assertEquals(72, localizer.pose().y(), EPS);
        assertEquals(0, localizer.pose().heading(), EPS);
    }

    @Test
    public void motionAfterSetPoseIsInTheNewFrame() {
        localizer.accept(0, 0, 0, 0, 0, 0);
        localizer.setPose(new Pose(0, 0, Math.PI / 2));
        localizer.accept(10, 0, 0, 4, 0, 0.1);
        assertEquals(0, localizer.pose().x(), EPS);
        assertEquals(10, localizer.pose().y(), EPS);
        assertEquals(4, localizer.velocity().vy, EPS);
        assertEquals(0.1, localizer.velocity().omega, EPS);
    }

    @Test
    public void setHeadingKeepsPosition() {
        localizer.accept(3, 4, 1, 0, 0, 0);
        localizer.setHeading(0);
        localizer.accept(3, 4, 1, 0, 0, 0);
        assertEquals(3, localizer.pose().x(), EPS);
        assertEquals(4, localizer.pose().y(), EPS);
        assertEquals(0, localizer.pose().heading(), EPS);
    }

    @Test
    public void staleUntilFirstSample() {
        assertFalse(localizer.hasData());
        assertTrue(Double.isInfinite(localizer.msSinceData()));
        localizer.accept(0, 0, 0, 0, 0, 0);
        assertTrue(localizer.hasData());
        assertTrue(localizer.msSinceData() < 1000);
    }
}
