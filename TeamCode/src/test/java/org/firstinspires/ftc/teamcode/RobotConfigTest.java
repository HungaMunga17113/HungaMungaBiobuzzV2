package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.Collections;

public class RobotConfigTest {
    @Test
    public void namesAndDirectionsMatchHardware() {
        assertEquals(Collections.emptyList(), RobotConfig.drift());
    }
}
