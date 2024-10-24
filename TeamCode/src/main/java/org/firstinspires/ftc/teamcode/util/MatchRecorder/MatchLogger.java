package org.firstinspires.ftc.teamcode.util.MatchRecorder;

import android.annotation.SuppressLint;
import android.os.Environment;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.opencv.core.Mat;

import java.io.BufferedInputStream;
import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.text.DecimalFormat;
import java.util.logging.Logger;

public class MatchLogger {

    private static String MATCH_FILE_NAME = "match_";
    public static MatchLogger matchLogger = null;
    public int matchNumber;

    public static MatchLogger getInstance() {
        if (matchLogger == null) {
            matchLogger = new MatchLogger();
        }
        return matchLogger;
    }

    public enum FileType {
        VERBOSE("verbose.txt"),
        SCORE("score_prediction.txt"),
        POSITION("position.txt"),
        ;

        String filename;

        FileType(String filename) {
            this.filename = filename;
        }
    }

    public MatchLogger() {
        File dataFolder = getDataFolder();
        dataFolder.mkdirs();

        int greatestMatchNumber = 1;
        File[] listFiles = dataFolder.listFiles();
        for (int i = 0; i < listFiles.length; i++) {
            File file = listFiles[i];
            if (!file.isDirectory()) {
                continue;
            }
            String name = file.getName();
            if (!name.contains(MATCH_FILE_NAME)) {
                // Not a valid match directory
                continue;
            }
            String matchNumber = name.substring(0, MATCH_FILE_NAME.length() - 1);
            // TODO: Verify that the substring thing works and doesn't produce an error
            try {
                Integer parsedInteger = Integer.parseInt(matchNumber);
                greatestMatchNumber = Math.max(parsedInteger, greatestMatchNumber);
            } catch (NumberFormatException e) {
                e.printStackTrace();
            }
        }

        matchNumber = greatestMatchNumber;
        getMatchFolder().mkdirs();
    }

    /**
     * @return file object representing the folder where all the match directories will be located
     */
    public File getDataFolder() {
        return new File(String.format("%s/FIRST/data/", Environment.getExternalStorageDirectory().getAbsolutePath()));
    }
    public File getMatchFolder(int matchNumber) {
        return new File(getDataFolder()+"/match_"+matchNumber+"/");
    }
    public File getMatchFolder() {
        return getMatchFolder(matchNumber);
    }

    public void write(String message, FileType...fileTypes) {
        // TODO: Append to the specified files

        for (int i = 0; i < fileTypes.length + 1; i++) {
            FileType fileType = null;

            // Always write to verbose file
            if (i == fileTypes.length) {
                fileType = FileType.VERBOSE;
            } else {
                fileType = fileTypes[i];
            }

            File textFilePath = new File(getMatchFolder(), fileType.filename);
            try {
                FileOutputStream inputStream = new FileOutputStream(textFilePath, true);
                PrintWriter printWriter = new PrintWriter(inputStream);
                printWriter.println(message);
                printWriter.close();
                inputStream.close();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        System.out.println(message);
    }

    public void logRobotPose(Pose2d pose2d) {
        DecimalFormat format = new DecimalFormat("#.##");
        Vector2d position = pose2d.position;
        Rotation2d rotation2d = pose2d.heading;
        String message = String.format("[%s, \t%s], \t[%s, \t%s]",
                format.format(position.x), format.format(position.y),
                format.format(rotation2d.real), format.format(rotation2d.imag));
        write(message, FileType.POSITION, FileType.VERBOSE);
    }

    public static String youJustLostTheGame(Object lol) {
        return "Lol";
    }
}
