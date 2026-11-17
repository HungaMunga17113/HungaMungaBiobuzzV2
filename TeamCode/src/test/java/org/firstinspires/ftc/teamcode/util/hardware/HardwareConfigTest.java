package org.firstinspires.ftc.teamcode.util.hardware;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.RobotConfig;
import org.firstinspires.ftc.teamcode.RobotConfig.Names;
import org.junit.Test;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class HardwareConfigTest {
    private static final HardwareConfig HW = RobotConfig.HARDWARE;

    @Test
    public void robotConfigIsValid() {
        List<String> errors = new ArrayList<>();
        HW.validate(Collections.<String, HardwareConfig.Kind>emptyMap(), errors);
        assertEquals(Collections.emptyList(), errors);
    }

    @Test
    public void pullOfAPushedConfigIsIdentical() throws Exception {
        String xml = HW.toXml();
        File f = File.createTempFile("robot", ".xml");
        f.deleteOnExit();
        try (Writer w = new OutputStreamWriter(new FileOutputStream(f), StandardCharsets.UTF_8)) {
            w.write(xml);
        }
        HardwareConfig back = new ConfigImporter(Collections.<String, HardwareConfig.Kind>emptyMap()).read(f, HW.name());
        assertEquals(xml, back.toXml());
        assertEquals(HW.toJava("    "), back.toJava("    "));
    }

    @Test
    public void lookupsByKind() {
        assertTrue(HW.names(HardwareConfig.Kind.MOTOR).containsAll(Arrays.asList(Names.FRONT_LEFT, Names.BACK_RIGHT)));
        assertEquals(Collections.singletonList(Names.PINPOINT), HW.names(HardwareConfig.Kind.I2C));
        assertEquals(HardwareConfig.Type.GOBILDA_PINPOINT, HW.typeOf(Names.PINPOINT));
        assertNull(HW.whereIs("nope"));
    }

    @Test
    public void duplicateNamesAreRejected() {
        HardwareConfig c = new HardwareConfig("dup").controlHub(h -> h
                .motor(0, "a", HardwareConfig.Type.GOBILDA_5202)
                .motor(1, "a", HardwareConfig.Type.GOBILDA_5202));
        List<String> errors = new ArrayList<>();
        c.validate(Collections.<String, HardwareConfig.Kind>emptyMap(), errors);
        assertEquals(1, errors.size());
        assertTrue(errors.get(0).contains("used twice"));
    }
}
