package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.util.ConfigUtil;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.util.Collections;

public class RobotConfigTest {
    @Test
    public void namesAndDirectionsMatchHardware() {
        assertEquals(Collections.emptyList(), ConfigUtil.drift());
    }
}
