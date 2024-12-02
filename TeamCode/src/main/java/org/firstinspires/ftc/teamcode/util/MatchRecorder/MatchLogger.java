package org.firstinspires.ftc.teamcode.util.MatchRecorder;

import android.os.Environment;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;

import java.io.File;
import java.io.FileOutputStream;
import java.io.PrintWriter;
import java.text.DecimalFormat;
import java.util.Objects;

public class MatchLogger {

    private static String MATCH_FILE_NAME = "match_";
    public static MatchLogger matchLogger = null;
    public int matchNumber;
    private Pose2d latestPose;

    public static final boolean REFLECTIONLESS = false; // turn to true if the robot is slow

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
        ARM("arm.txt"),
        FINGER("finger.txt"),
        WRIST("wrist.txt"),
        LAUNCHER("launcher.txt"),
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
            String matchNumber = name.substring(MATCH_FILE_NAME.length(), name.length() - 1);
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

    /**
     * Records the current robot position
     */
    // TODO: there is no framework for logging positions in the DriveSubsystem for normal driving (no odometry)
    public void logRobotPose(Pose2d pose2d) {
        DecimalFormat format = new DecimalFormat("#.##");
        Vector2d position = pose2d.position;
        Rotation2d rotation2d = pose2d.heading;
        String message = String.format("[%s, \t%s], \t[%s, \t%s]",
                format.format(position.x), format.format(position.y),
                format.format(rotation2d.real), format.format(rotation2d.imag));
        write(message, FileType.POSITION, FileType.VERBOSE);

        // Record the latest pose for teleop when auto is finished.
        latestPose = pose2d;
    }

    private String arrayToString(Object[] relevantVariables) {
        if (relevantVariables == null) {
            return "";
        }
        String message = "";
        for (int i = 0; i < relevantVariables.length; i++) {
            message += Objects.toString(relevantVariables[i]);
            if (i != relevantVariables.length - 1) {
                message += " ";
            }
        }
        return message;
    }

    /**
     * Records the passed variables and logs the method that was called before this.
     */
    /*public void logArm(ArmSubsystem subsystem, Object...relevantVariables) {
        String message = String.format("Arm: %s | %s", getCalledMethodName(), arrayToString(relevantVariables));
        write(message, FileType.ARM, FileType.VERBOSE);
    }*/

    public void genericLog(String header, FileType fileType, Object...relevantVariables) {
        String message = String.format("%s: %s", getCalledMethodName(), arrayToString(relevantVariables));
        write(message, fileType, FileType.VERBOSE);
    }

    private String getCalledMethodName() {
        if (REFLECTIONLESS) {
            return "[reflectionless]";
        }
        StackTraceElement[] elements = new Throwable().getStackTrace();
        if (elements == null || elements.length == 0) {
            return "[err_no_stacktrace]";
        }
        for (int i = 0; i < elements.length; i++) {
            String calledClassName = elements[i].getClassName();
            if (calledClassName.equals(getClass().getName())) {
                continue;
            }
            // It is something other than the MatchLogger, record the method used
            return String.format("[%s]", elements[i].getMethodName());
        }
        return "[err_404]"; // not found
    }

    public static String youJustLostTheGame(Object lol) {
        return "Lol";
    }
}
