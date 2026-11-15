package org.firstinspires.ftc.teamcode.util.tuning;

import android.content.Context;

import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;

import fi.iki.elonen.NanoHTTPD;
import fi.iki.elonen.NanoHTTPD.Response.Status;

/** http://192.168.43.1:8080/tuning/sva (needs a full install, not sloth, to pick up changes) */
public final class TuningServer {
    private TuningServer() {}

    @WebHandlerRegistrar
    public static void attach(Context context, WebHandlerManager manager) {
        final Context app = context.getApplicationContext();
        WebHandler page = session -> {
            try (InputStream in = app.getAssets().open("tuning/sva.html")) {
                return respond(Status.OK, "text/html", new String(read(in), StandardCharsets.UTF_8));
            }
        };
        manager.register("/tuning/sva", page);
        manager.register("/tuning/sva/", page);
        manager.register("/tuning/sva/runs", session -> {
            StringBuilder sb = new StringBuilder("[");
            List<String> runs = TuningFiles.runs();
            for (int i = 0; i < runs.size(); i++) sb.append(i == 0 ? "" : ",").append('"').append(runs.get(i)).append('"');
            return respond(Status.OK, "application/json", sb.append(']').toString());
        });
        manager.register("/tuning/sva/run", session -> {
            Map<String, List<String>> params = session.getParameters();
            List<String> name = params.get("name");
            File f = TuningFiles.run(name == null || name.isEmpty() ? null : name.get(0));
            if (f == null) return respond(Status.NOT_FOUND, NanoHTTPD.MIME_PLAINTEXT, "no such run");
            try (InputStream in = new FileInputStream(f)) {
                return respond(Status.OK, "application/json", new String(read(in), StandardCharsets.UTF_8));
            }
        });
    }

    private static NanoHTTPD.Response respond(Status status, String mime, String body) {
        NanoHTTPD.Response r = NanoHTTPD.newFixedLengthResponse(status, mime, body);
        r.addHeader("Cache-Control", "no-store");
        return r;
    }

    private static byte[] read(InputStream in) throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        byte[] buf = new byte[8192];
        for (int n; (n = in.read(buf)) > 0; ) out.write(buf, 0, n);
        return out.toByteArray();
    }
}
