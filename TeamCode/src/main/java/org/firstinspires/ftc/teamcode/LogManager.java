/*
 * Base code to manage the robot. Baseline copied from FtcRobotController sample code
 */

package org.firstinspires.ftc.teamcode;

import com.google.blocks.ftcrobotcontroller.util.FileManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LogManager {

    private Telemetry telemetry;


    private boolean fileLoggingEnabled = true;

    private static String logFileFolder = "C:\\FTC\\Logs\\";

    private File logFile;

    public LogManager(Telemetry telemetry, String OpModeName) {

        this.telemetry = telemetry;
        if (fileLoggingEnabled)
            CreateNewLogfile(OpModeName);

    }

    private String GetTimeStampString()
    {
        return new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
    }

    private void CreateNewLogfile(String OpModeName) {
        String fileName = OpModeName + "_" + GetTimeStampString() + ".txt";
        logFile = new File(logFileFolder + fileName);
        if (!logFile.exists()) logFile.mkdirs();

    }

    public void WriteLog(String Caption, String Text) {
        this.telemetry.addData(Caption, Text);
        this.telemetry.update();

        if (fileLoggingEnabled)
            WriteToFile(GetTimeStampString() + ":" + Caption + " --- " + Text);

    }

    private void WriteToFile(String Text) {
        try {
            FileWriter fileWriterObj= new FileWriter(logFile, true);
            fileWriterObj.write(Text);
        }
        catch(Exception Ex) {
            this.telemetry.addData("File Writer Error: ", Ex.getMessage());
        }
    }
}
