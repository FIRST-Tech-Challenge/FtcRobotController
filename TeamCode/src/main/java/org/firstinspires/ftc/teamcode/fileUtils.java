package org.firstinspires.ftc.teamcode;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;

public class fileUtils {

    public class Param {
        Pose2d pose;
    }

    private String configFileName = "FtcRobotConfig.txt";
    public Param param;

    public fileUtils() {
        param = new Param();
    }

    public void setPose(Pose2d pose) {
        param.pose = pose;
    }
    public Pose2d getPose() {
        return param.pose;
    }
    public void writeConfig(Context context, OpMode opMode) {
        try {
            OutputStreamWriter outputStreamWriter = new OutputStreamWriter(context.openFileOutput(configFileName, Context.MODE_PRIVATE));

            // write each configuration parameter as a string on its own line
            outputStreamWriter.write(Double.toString(param.pose.getX())+"\n");
            outputStreamWriter.write(Double.toString(param.pose.getY())+"\n");
            outputStreamWriter.write(Double.toString(param.pose.getHeading())+"\n");
            outputStreamWriter.close();
        }
        catch (IOException e) {
            opMode.telemetry.addData("Exception", "Configuration file write failed: " + e.toString());
        }
    }

    public void readConfig(Context context, OpMode opMode) {

        // read configuration data from file
        try {
            InputStream inputStream = context.openFileInput(configFileName);

            if (inputStream != null) {
                InputStreamReader inputStreamReader = new InputStreamReader(inputStream);
                BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

                Double x, y, heading;
                x = Double.valueOf(bufferedReader.readLine());
                y = Double.valueOf(bufferedReader.readLine());
                heading = Double.valueOf(bufferedReader.readLine());
                param.pose = new Pose2d(x,y,heading);

                inputStream.close();
            }
        } catch (Exception e) {
            opMode.telemetry.addData("Exception", "Error reading config file: " + e.toString());
            // can't read from file, so initialize to reasonable values
            param.pose = new Pose2d(0.0, 0.0, 0.0);
        }

    }
}