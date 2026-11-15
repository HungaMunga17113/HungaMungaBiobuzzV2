package org.firstinspires.ftc.teamcode.util.tuning;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import java.util.regex.Pattern;

/** sva runs on the hub, /sdcard/FIRST/tuning/sva */
public final class TuningFiles {
    public static final File SVA_DIR = new File(AppUtil.FIRST_FOLDER, "tuning/sva");
    private static final Pattern NAME = Pattern.compile("sva-\\d{8}-\\d{6}\\.json");

    private TuningFiles() {}

    public static File save(String json) throws IOException {
        if (!SVA_DIR.isDirectory() && !SVA_DIR.mkdirs()) throw new IOException("can't create " + SVA_DIR);
        String stamp = new SimpleDateFormat("yyyyMMdd-HHmmss", Locale.US).format(new Date());
        File f = new File(SVA_DIR, "sva-" + stamp + ".json");
        try (Writer w = new OutputStreamWriter(new FileOutputStream(f), StandardCharsets.UTF_8)) {
            w.write(json);
        }
        return f;
    }

    /** newest first */
    public static List<String> runs() {
        List<String> out = new ArrayList<>();
        String[] names = SVA_DIR.list();
        if (names != null) for (String n : names) if (NAME.matcher(n).matches()) out.add(n);
        Collections.sort(out, Collections.<String>reverseOrder());
        return out;
    }

    /** null unless it's a real run file */
    public static File run(String name) {
        if (name == null || !NAME.matcher(name).matches()) return null;
        File f = new File(SVA_DIR, name);
        return f.isFile() ? f : null;
    }
}
