package org.firstinspires.ftc.teamcode.tuning;

import android.content.Context;
import android.content.res.AssetManager;

import com.fasterxml.jackson.core.JsonFactory;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.WebHandlerManager;

import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Objects;

import fi.iki.elonen.NanoHTTPD;

public final class TuningFiles {
    private static final File ROOT =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/tuning/");

    private static final Object IO_LOCK = new Object();

    private static WebHandlerManager whm; // guarded by ioLock

    public enum FileType {
        FORWARD_RAMP("forward-ramp"),
        ANGULAR_RAMP("angular-ramp"),
        ACCEL("accel");

        public final String name;

        FileType(String s) {
            name = s;
        }
    }

    private TuningFiles() {

    }

    private static File getFileTypeDir(FileType ty) {
        return new File(ROOT, ty.name);
    }

    public static void save(FileType ty, Object data) {
        synchronized (IO_LOCK) {
            File f = new File(getFileTypeDir(ty), System.currentTimeMillis() + ".json");

            try {
                new ObjectMapper(new JsonFactory())
                        .writerWithDefaultPrettyPrinter()
                        .writeValue(f, data);

                if (whm != null) {
                    whm.register("/tuning/" + ty.name + "/" + f.getName(),
                            newStaticFileHandler(f));
                }
            } catch (IOException e) {
                RobotLog.setGlobalErrorMsg(new RuntimeException(e),
                        "Unable to write data to " + f.getAbsolutePath());
            }
        }
    }

    @WebHandlerRegistrar
    public static void registerRoutes(Context context, WebHandlerManager manager) {
        synchronized (IO_LOCK) {
            AssetManager assetManager = context.getAssets();
            registerAssetsUnderPath(manager, assetManager, "tuning");
            for (FileType ty : FileType.values()) {
                String base = "/tuning/" + ty.name;

                WebHandler wh = newStaticAssetHandler(assetManager, ty.name + "/index.html");
                manager.register(base, wh);
                manager.register(base + "/", wh);

                File dir = getFileTypeDir(ty);
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();

                manager.register(base + "/latest.json",
                        newLatestFileHandler(dir));
                for (File f : Objects.requireNonNull(dir.listFiles())) {
                    manager.register(base + "/" + f.getName(),
                            newStaticFileHandler(f));
                }
            }

            whm = manager;
        }
    }

    private static WebHandler newStaticAssetHandler(AssetManager assetManager, String file) {
        return session -> {
            if (session.getMethod() == NanoHTTPD.Method.GET) {
                String mimeType = MimeTypesUtil.determineMimeType(file);
                return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        mimeType, assetManager.open(file));
            } else {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "");
            }
        };
    }

    private static void registerAssetsUnderPath(WebHandlerManager webHandlerManager,
                                                AssetManager assetManager, String path) {
        try {
            String[] list = assetManager.list(path);

            if (list == null) return;

            if (list.length > 0) {
                for (String file : list) {
                    registerAssetsUnderPath(webHandlerManager, assetManager, path + "/" + file);
                }
            } else {
                webHandlerManager.register(path, newStaticAssetHandler(assetManager, path));
            }
        } catch (IOException e) {
            RobotLog.setGlobalErrorMsg(new RuntimeException(e),
                    "unable to register tuning web routes");
        }
    }


    private static WebHandler newLatestFileHandler(File dir) {
        return session -> {
            File[] files = dir.listFiles();
            if (files != null) {
                long mostRecentLastModified = 0;
                File mostRecentFile = null;
                for (File f : files) {
                    long lastModified = f.lastModified();
                    if (lastModified > mostRecentLastModified) {
                        mostRecentLastModified = lastModified;
                        mostRecentFile = f;
                    }
                }

                if (mostRecentFile != null) {
                    String mimeType = MimeTypesUtil.determineMimeType(mostRecentFile.getName());
                    final NanoHTTPD.Response res = NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                            mimeType,
                            new FileInputStream(mostRecentFile));
                    res.addHeader("X-Filename", mostRecentFile.getName());
                    return res;
                }
            }

            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                    NanoHTTPD.MIME_PLAINTEXT, "");
        };
    }

    private static WebHandler newStaticFileHandler(File f) {
        return session -> {
            if (session.getMethod() == NanoHTTPD.Method.GET) {
                String mimeType = MimeTypesUtil.determineMimeType(f.getName());
                return NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        mimeType, new FileInputStream(f));
            } else {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                        NanoHTTPD.MIME_PLAINTEXT, "");
            }
        };
    }
}
