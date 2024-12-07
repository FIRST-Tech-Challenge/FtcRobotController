package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class
FileRead {
    // File reading stuff
    File file;
    Scanner scan;
    String fileName = "/sdcard/FIRST/PathTest.txt";
    String position;
    int positions = 0;
    String[][] path;
    double dYdX;
    double totalDistance;
    double distance;
    double theta;
    double xCoordinate;
    double yCoordinate;
    double prevReadPressTime = 0;

    ElapsedTime totalTime = new ElapsedTime();

    OpMode master;

    public void init(OpMode opMode)
    {
        master = opMode;
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void readFile()
    {
        if (master.gamepad1.y && totalTime.milliseconds() - 500 > prevReadPressTime) {
            prevReadPressTime = totalTime.milliseconds();

            file = new File(fileName);
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            while (scan.hasNextLine()) {
                positions++;
                scan.nextLine();
            }
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            path = new String[positions][7];
            for (int i = 0; i < path.length; i++) {
                if (scan.hasNextLine()) {
                    position = scan.nextLine();
                    position = position.replaceAll("[^0-9. -]", "");
                    path[i] = position.split(" ");
                }
                try {
                    dYdX = Double.parseDouble(path[i][1]);
                    totalDistance = Double.parseDouble(path[i][2]);
                    distance = Double.parseDouble(path[i][3]);
                    theta = Double.parseDouble(path[i][4]);
                    xCoordinate = Double.parseDouble(path[i][5]);
                    yCoordinate = Double.parseDouble(path[i][6]);
                } catch (NumberFormatException e) {
                    continue;
                }
                master.telemetry.addLine("dYdX = " + dYdX);
                master.telemetry.addLine("totalDist = " + totalDistance);
                master.telemetry.addLine("dist = " + distance);
                master.telemetry.addLine("theta = " + theta);
                master.telemetry.addLine("x = " + xCoordinate);
                master.telemetry.addLine("y = " + yCoordinate);
                master.telemetry.addLine(" ");
            }
            master.telemetry.update();
        }
    }




}
