package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;

public class LoggingUtil {
    public static File getLogRoot(Context context) {
        File dir = new File(Environment.getExternalStorageDirectory(), "ACME");
        dir.mkdirs();
        return dir;
    }

    public static File getLogRoot(OpMode opMode) {
        return getLogRoot(opMode.hardwareMap.appContext);
    }

    private static void removeRecursive(File file) {
        if (file.isDirectory()) {
            for (File childFile : file.listFiles()) {
                removeRecursive(childFile);
            }
        }
        file.delete();
    }

    public static void clearLogs(Context context) {
        removeRecursive(getLogRoot(context));
    }
}
