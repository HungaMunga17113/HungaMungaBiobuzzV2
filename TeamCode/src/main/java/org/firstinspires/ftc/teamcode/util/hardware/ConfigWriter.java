package org.firstinspires.ftc.teamcode.util.hardware;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * checks RobotConfig.HARDWARE and write it to an xml
 * usage:
 * ConfigWriter &lt;output dir&gt; [KIND=xmlTag ...]
 * where KIND=xmlTag pairs are device types declared in TeamCode
 */
public final class ConfigWriter {
    public static void main(String[] args) throws Exception {
        HardwareConfig config;
        try {
            // so this tool compiles without RobotConfig
            config = (HardwareConfig) Class.forName("org.firstinspires.ftc.teamcode.RobotConfig")
                    .getField("HARDWARE").get(null);
        } catch (ExceptionInInitializerError e) {
            System.err.println("RobotConfig failed to load: " + e.getCause());
            System.exit(1);
            return;
        }

        Map<String, HardwareConfig.Kind> custom = customTypes(args, 1);
        List<String> errors = new ArrayList<>();
        List<String> warnings = config.validate(custom, errors);
        for (String w : warnings) System.out.println("warning: " + w);
        if (!errors.isEmpty()) {
            System.err.println("RobotConfig.HARDWARE has " + errors.size() + " problem" + (errors.size() == 1 ? "" : "s") + ":");
            for (String e : errors) System.err.println("  " + e);
            System.exit(1);
        }

        File out = new File(args[0], config.name() + ".xml");
        out.getParentFile().mkdirs();
        try (Writer w = new OutputStreamWriter(new FileOutputStream(out), StandardCharsets.UTF_8)) {
            w.write(config.toXml(custom));
        }
        System.out.println(out.getAbsolutePath());
    }

    static Map<String, HardwareConfig.Kind> customTypes(String[] args, int from) {
        Map<String, HardwareConfig.Kind> custom = new LinkedHashMap<>();
        for (int i = from; i < args.length; i++) {
            int eq = args[i].indexOf('=');
            custom.put(args[i].substring(eq + 1), HardwareConfig.Kind.valueOf(args[i].substring(0, eq)));
        }
        return custom;
    }
}
