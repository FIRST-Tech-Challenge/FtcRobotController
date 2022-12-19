package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive")
public class Drive extends Control {

    @Override
    public void init() {
        super.init();
        // Commented this out for testing, but you don't need this if you are doing this in Hraezlyr anyways
/*
        hraezlyr.cascadeMotor1.setPower(0);
        hraezlyr.cascadeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hraezlyr.cascadeMotor2.setPower(0);
        hraezlyr.cascadeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);*/
    }

    @Override
    public void start() {
        super.start();
        hraezlyr.resetIMU();
    }

    @Override
    public void loop() {
        double leftX = gamepad1.left_stick_x;
        double leftY = gamepad1.left_stick_y;
        double rightX = gamepad1.right_stick_x;

        Level zHeight = Level.GROUND;
        boolean start = gamepad1.start;
        boolean DpadUp = gamepad2.dpad_up;
        boolean DpadDown = gamepad2.dpad_down;
        boolean DpadLeft = gamepad2.dpad_left;
        boolean DpadRight = gamepad2.dpad_right;
        boolean DpadPressed;
        double rightTurn = 0;
        double leftTurn = 0;
        double resetIMU = 0;

        // changes power based on direction of turn to turn robot instead of strafe
        if (start) {
            resetIMU = hraezlyr.resetIMU();
        }
        // angle of controller stick
        if (rightX > 0) {
            rightTurn = rightX;
        }
        if (rightX < 0) {
            leftTurn = rightX;
        }
        double angle = Math.toDegrees(Math.atan2(leftY, leftX));
        angle = angle - hraezlyr.getHeading() + resetIMU;
        angle = constrainAngle(angle);
        // scope orientation
        // distance is the power of the controller stick
        double distance = Math.max(-1, Math.min(1, Math.sqrt((leftX * leftX) + (leftY * leftY))));


        // topLeftPower and bottomRightPower
        double powerGroup1 = ((Math.sin(Math.toRadians(angle))) - (Math.cos(Math.toRadians(angle)))) * distance;
        // topRightPower and bottomLeftPower
        double powerGroup2 = ((Math.sin(Math.toRadians(angle))) + (Math.cos(Math.toRadians(angle)))) * distance;

        // Power for drivetrain
        hraezlyr.topLeft.setPower(powerGroup1 - leftTurn + rightTurn);
        hraezlyr.topRight.setPower(powerGroup2 - rightTurn + leftTurn);
        hraezlyr.bottomLeft.setPower(powerGroup2 - rightTurn +leftTurn);
        hraezlyr.bottomRight.setPower(powerGroup1 - leftTurn +rightTurn);

        telemetry.addData("Drive power1", powerGroup1);
        telemetry.addData("Drive power2", powerGroup2);
        telemetry.addData("Angle", angle);
        telemetry.addData("Distance", distance);
        //telemetry.addData("Cascade Height", hraezlyr.cascadeMotor1.getCurrentPosition());
        telemetry.update();

        //System for cascade level system

        if(gamepad2.dpad_right){
            switch(zHeight){// it go up if it already low
                case GROUND:
                    zHeight = Level.LOW;
                    break;
                case LOW:
                    zHeight = Level.MEDIUM;
                    break;
                case MEDIUM:
                    zHeight = Level.HIGH;
                    break;
            }
            cascadeLift(zHeight);
        }
            if(gamepad2.dpad_left){
            switch(zHeight) {//it go down if already up
                case HIGH:
                    zHeight = Level.MEDIUM;
                    break;
                case MEDIUM:
                    zHeight = Level.LOW;
                    break;
                case LOW:
                    zHeight = Level.GROUND;
                    break;


            }
        }

            if(gamepad2.dpad_up) cascadeLiftManual(1);
            else if(gamepad2.dpad_down) cascadeLiftManual(-1);
            else cascadeLiftManual(0);

        }



    }
