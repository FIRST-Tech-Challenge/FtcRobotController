package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Autonomous extends Control {
    Pipeline pipeline = new Pipeline();

    // diagonal fov of camera 55 degrees
    // horizontal fov of camera is 49 degrees
    // vertical fov 28 fov
    // 1280 by 720
    // 16:9 aspect ratio

    // TODO: fix ticks per inch to be accurate
    public static final double TICKS_PER_INCH = 4096;

    @Override
    public void loop() {
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
            hraezlyr.topLeft.setPower(powerGroup1 * speed + turn);
            hraezlyr.topRight.setPower(powerGroup2 * speed - turn);
            hraezlyr.bottomLeft.setPower(powerGroup2 * speed - turn);
            hraezlyr.bottomRight.setPower(powerGroup1 * speed + turn);

    }
    public void driveEncoder(double x, double y, double power) throws InterruptedException {
        int topLeft = hraezlyr.topLeft.getCurrentPosition();
        int topRight = hraezlyr.topRight.getCurrentPosition() ;
        int bottomLeft = hraezlyr.bottomLeft.getCurrentPosition() ;
        int bottomRight = hraezlyr.bottomRight.getCurrentPosition() ;

        // rotates by 45 to line up with mecanum wheels
        double imuAngle = hraezlyr.getHeading() - (Math.PI/2);

        // calculates given x and y into robots perspective of x and y also converts to ticks
        int x_output = (int)(((x * Math.cos(imuAngle)) + (y * Math.sin(imuAngle))) * TICKS_PER_INCH);
        int y_output = (int)(((x * (-Math.sin(imuAngle))) + (y * Math.cos(imuAngle))) * TICKS_PER_INCH);

        topLeft +=  x_output;
        bottomRight += x_output;
        topRight += y_output;
        bottomLeft += y_output;

        hraezlyr.setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

        hraezlyr.topLeft.setTargetPosition(topLeft);
        hraezlyr.topRight.setTargetPosition(topRight);
        hraezlyr.bottomLeft.setTargetPosition(bottomLeft);
        hraezlyr.bottomRight.setTargetPosition(bottomRight);

        hraezlyr.topLeft.setPower(x_output);
        hraezlyr.topRight.setPower(y_output);
        hraezlyr.bottomLeft.setPower(x_output);
        hraezlyr.bottomRight.setPower(y_output);

        while(hraezlyr.topLeft.isBusy() || hraezlyr.topRight.isBusy() || hraezlyr.bottomLeft.isBusy() || hraezlyr.bottomRight.isBusy());{
            Thread.sleep(10);
        }

    }
}