package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

public abstract class AutoGuts extends Control {

    ColorDetection colorDetection;
    Pipeline pipeline;

    @Override
    public void loop() {
        requestOpModeStop();
    }

    // TODO: fix ticks per inch to be accurate
    public static final double TICKS_PER_INCH = 4096;

    @Override
    public void init() {
        super.init();
        colorDetection = new ColorDetection();
        pipeline = new Pipeline();
    }

    public void driveEncoder(double x, double y, double power) {

        int topLeft = hraezlyr.topLeft.getCurrentPosition();
        int topRight = hraezlyr.topRight.getCurrentPosition();
        int bottomLeft = hraezlyr.bottomLeft.getCurrentPosition();
        int bottomRight = hraezlyr.bottomRight.getCurrentPosition();

        // rotates by 45 to line up with mecanum wheels
        double imuAngle = hraezlyr.getHeading() - (180 / 2);

        // calculates given x and y into robots perspective of x and y also converts to ticks
        int x_output = (int) (((x * Math.cos(imuAngle)) + (y * Math.sin(imuAngle))) * TICKS_PER_INCH);
        int y_output = (int) (((x * (-Math.sin(imuAngle))) + (y * Math.cos(imuAngle))) * TICKS_PER_INCH);

        topLeft += x_output;
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

        while (hraezlyr.topLeft.isBusy() || hraezlyr.topRight.isBusy() || hraezlyr.bottomLeft.isBusy() || hraezlyr.bottomRight.isBusy()) {
            sleep(10);
           hraezlyr.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }

    public void autoDrive(double power, double angle, double turn) {
        double powerGroup1 = 0;
        double powerGroup2 = 0;

        powerGroup2 = ((-Math.sin(angle)) + (Math.cos(angle))) * power;
        powerGroup1 = ((Math.cos(angle) + Math.sin(angle))) * power;

        hraezlyr.topLeft.setPower(powerGroup1 + turn);
        hraezlyr.topRight.setPower(powerGroup2 - turn);
        hraezlyr.bottomLeft.setPower(powerGroup2 + turn);
        hraezlyr.bottomRight.setPower(powerGroup1 - turn);

    }

    public void turn(double power, double angle) {
        double angularDistance = 0;
        do {
            double initialAngle = hraezlyr.getHeading();
            angle = constrainAngle(angle);
            double turnVal = 1;

            if (angle - initialAngle < 0) turnVal = -1;
            //counter clockwise
            angularDistance = Math.abs(angle - initialAngle);
            if (angularDistance > 180) { // dealing with edge cse
                turnVal *= -1;
                angularDistance =  360 - angularDistance; // calculating shorter angularDistance
            }
            turnVal = turnVal * angularDistance / .35;
            hraezlyr.topLeft.setPower(turnVal);
            hraezlyr.topRight.setPower(-turnVal);
            hraezlyr.bottomLeft.setPower(turnVal);
            hraezlyr.bottomRight.setPower(-turnVal);
        }
        while (!isStopRequested && angularDistance > 0.08);
    }



    public void closeClaw(boolean clawClose){

        double close = 0;
        if (clawClose) close = 0;
        if (!clawClose) close = 50;

        hraezlyr.servoClaw.setPosition(close);

    }

}
