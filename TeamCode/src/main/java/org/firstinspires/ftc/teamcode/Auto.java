package org.firstinspires.ftc.teamcode;

public class Auto extends AutoGuts{
    Pipeline pipeline;
    // diagonal fov of camera 55 degrees
    // horizontal fov of camera is 49 degrees
    // vertical fov 28 fov
    // 1280 by 720
    // 16:9 aspect ratio

    @Override
    public void start() {
        double midPointX = pipeline.getRectMidpointX();
        double width = pipeline.getRectWidth();
        double area = pipeline.getRectArea();
        double turn = 0;
        double powerGroup1 = 0;
        double powerGroup2 = 0;
        double speed = 0;
        double angleRatio = 0;
        double angle = 0;
        double loopCounter = 0;

        //TODO: code doesn't actually do much..... fix it

        // makes robot turn if no box is detected
        if(area == 0){
            turn = 1;
        }
        if(area != 0) {
            turn = 0;
            // gets ratio of 1 to 0 of the screen
            angleRatio = (width - midPointX) / width;
            // multiplies by 180 to get the angle in the polar coordinates
            angle = angleRatio * Math.PI;

            // bigger area is slower speed
            if (area > 1000) {
                speed = 1000 / area;
            }


            // topLeftPower and bottomRightPower
            powerGroup1 = (Math.sin(angle) - Math.cos(angle));
            // topRightPower and bottomLeftPower
            powerGroup2 = (Math.sin(angle) + Math.cos(angle));
        }
        // Power for drivetrain

    }
}
