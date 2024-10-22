package org.firstinspires.ftc.teamcode.util.MatchRecorder;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;

public class MatchLogger {

    public static class FileLogger {
        static int matchNumber = 0;
        @SuppressLint("DefaultLocale")
        final String logFilePath = String.format("%s/FIRST/data/%d.txt", Environment.getExternalStorageDirectory().getAbsolutePath(), matchNumber);

    }

    public static MatchLogger matchLogger = null;

    public static MatchLogger getInstance() {
        if (matchLogger == null)
            matchLogger = new MatchLogger();

        return matchLogger;
    }

    public void LogRobotPose(Pose2d pose2d) {

    }

    public static String youJustLostTheGame(Object lol) {
        return "Lol";
    }
}
