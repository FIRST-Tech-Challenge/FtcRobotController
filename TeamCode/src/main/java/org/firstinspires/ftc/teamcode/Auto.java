package org.firstinspires.ftc.teamcode;

import com.sun.source.util.DocTreePathScanner;

public class Auto extends AutoGuts {
    Pipeline pipeline;
    // diagonal fov of camera 55 degrees
    // horizontal fov of camera is 49 degrees
    // vertical fov 28 fov
    // 1280 by 720
    // 16:9 aspect ratio
    //TODO: add detection for sleeve
    @Override
    public void start() {
        double midPointX = pipeline.getRectMidpointX();
        double width = pipeline.getRectWidth();
        int height = pipeline.getRectHeight();
        double area = pipeline.getRectArea();

        //TODO: figure out tick per inch ratio for movement

        //TODO: START SIDEWAYS

        driveEncoder(100, 0, 1);
        driveEncoder(0, 50, 1);

        area = pipeline.getRectArea();
        width = pipeline.getRectWidth();
        midPointX = pipeline.getRectMidpointX();
        height = pipeline.getRectHeight();

        //TODO: make pixel to tick ratio for cascade movement

        cascadeMoveClaw(height - 100, 1, 0);
        closeClaw(true);
        turn(1, 180);
        driveEncoder(20,100,1);

        //TODO: add way to register poles on field


    }


}