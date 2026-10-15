package org.firstinspires.ftc.teamcode.utils;

import android.os.Environment;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.RobotLog;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

public class PoseStorage {

    private static final String FILE_NAME = "lastPose.txt";

    //save the pose
    public static void savePose(Pose pose) {
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/" + FILE_NAME);

        try (PrintWriter writer = new PrintWriter(new FileWriter(file))) {
            writer.println(pose.getX());
            writer.println(pose.getY());
            writer.println(pose.getHeading());
        } catch (IOException e) {
            RobotLog.ee("PoseStorage", "Failed to save pose", e);
        }
    }

    //load the pose
    public static Pose loadPose() {
        File file = new File(Environment.getExternalStorageDirectory().getPath() + "/" + FILE_NAME);

        if (!file.exists()) {
            return new Pose(67, 67, 0);
        }

        try (BufferedReader reader = new BufferedReader(new FileReader(file))) {
            double x = Double.parseDouble(reader.readLine());
            double y = Double.parseDouble(reader.readLine());
            double heading = Double.parseDouble(reader.readLine());

            return new Pose(x, y, heading);
        } catch (Exception e) {
            RobotLog.ee("PoseStorage", "Failed to load pose", e);
            return new Pose(69, 69, 0);
        }
    }
}