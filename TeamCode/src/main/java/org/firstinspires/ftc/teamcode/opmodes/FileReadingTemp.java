package org.firstinspires.ftc.teamcode.opmodes;

import android.os.Environment;
import android.util.Log;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Scanner;

public class FileReadingTemp {

    //TODO: Fix code for use with Swerve Drive; this is a temp location for this code
    void readInstructionFile(String filename) {
        File file = new File(Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS) + "/" + filename);
        Scanner scanner = null;
        try {
            scanner = new Scanner(file);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        ArrayList<String> data = new ArrayList<>();
        if (scanner != null) {
            while (scanner.hasNext()) {
                data.add(scanner.nextLine());
            }
        }

        for (String s : data) {
            float x;
            float y;
            int theta;
            int time;
            double speed;

            float xOffset;
            float yOffset;
            String target;

            String[] t0 = s.split(":");
            switch (t0[0]) {
                case "s": {
                    String[] t1 = t0[1].split(",");
                    x = Float.parseFloat(t1[0]);
                    y = Float.parseFloat(t1[1]);
                    theta = Integer.parseInt(t1[2]);

                    Log.i("ROBOT", String.format("Start: x=%f, y=%f, t=%d", x, y, theta));

                    //This is to setup where the robot starts
                    RobotObject obj = new RobotObject(x, y, 18, 18, theta);
                    resetRobotObject(obj);
                    break;
                }
                case "c": {
                    String[] t1 = t0[1].split(",");
                    x = Float.parseFloat(t1[0]);
                    y = Float.parseFloat(t1[1]);
                    theta = Integer.parseInt(t1[2]);
                    theta -= robot.driveController.getRotation();
                    time = Integer.parseInt(t1[3]);
                    speed = Double.parseDouble(t1[4]);

                    Log.i("ROBOT", "Start Pos: " + robotObject.getCenterX() + " " + robotObject.getCenterY());

                    Log.i("ROBOT", String.format("Coord: x=%f, y=%f, t=%d, m=%d", x, y, theta, time));

                    //This is to actually move it to a position
                    moveCoord(x, y, theta, 100, time, speed);
                    break;
                }
                case "t": {
                    Log.i("ROBOT", t0[1]);
                    String[] t1 = t0[1].split(",");
                    target = t1[0];
                    xOffset = Float.parseFloat(t1[1]);
                    yOffset = Float.parseFloat(t1[2]);
                    theta = Integer.parseInt(t1[3]);
                    theta -= robot.driveController.getRotation();
                    time = Integer.parseInt(t1[4]);
                    speed = Double.parseDouble(t1[5]);

                    Log.i("ROBOT", "Start Pos: " + robotObject.getCenterX() + " " + robotObject.getCenterY());

                    Log.i("ROBOT", String.format("Start: target=%s x=%f, y=%f, t=%d, m=%d", target, xOffset, yOffset, theta, time));

                    //This is to actually move it to a target
                    moveTarget(field.getObject(target), xOffset, yOffset, theta, 100, time, speed);
                    break;
                }
            }
        }
    }

}
