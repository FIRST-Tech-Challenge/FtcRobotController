package org.firstinspires.ftc.teamcode.Debug;
import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class Debug {
    public boolean debugMode = false;
    private final OpMode opMode;
    private final Telemetry telemetry;

    private File logFile;

    public Debug(OpMode opMode) {
        this.opMode = opMode;
        this.telemetry = opMode.telemetry;

        try {
            // Get the directory for public external storage
            File logDir = new File(Environment.getExternalStorageDirectory(), "FTCLogs");
            if (!logDir.exists()) {
                logDir.mkdirs(); // Create the directory if it doesn't exist
            }

            // Get the current date and time
            SimpleDateFormat formatter = new SimpleDateFormat("MM-dd-yyyy HH:mm:ss");
            String formattedDateTime = formatter.format(new Date());

            // Get the class name of the file the Debug class was instantiated from
            String className = opMode.getClass().getSimpleName();

            // Create a new log file with the specified format
            String logFileName = className + " " + formattedDateTime + ".txt";
            logFile = new File(logDir, logFileName);
            if (!logFile.exists()) {
                logFile.createNewFile();
            }

            log("Log file created");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void checkDebugButtons(Gamepad gamepad) {
        if (gamepad.start && gamepad.back) {
            debugMode = !debugMode;
            telemetry.addData("debug mode: ", debugMode);
            telemetry.update();
        }
    }

    public void log(String message) {
        try {
            FileWriter writer = new FileWriter(logFile, true);
            writer.write(message + " ");
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String getLogFilePath() {
        return logFile.getAbsolutePath();
    }

    public boolean getDebugMode() {
        return debugMode;
    }

    public void outputDebugInfo(String message) {
        if (debugMode) {
            telemetry.addData("Debug Info: ", message);
        } else {
            telemetry.addData("Debug Info: ", "Debug mode is off");
        }
        telemetry.update();
    }
}