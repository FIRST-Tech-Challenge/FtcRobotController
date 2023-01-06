package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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
        boolean dpadLeft = gamepad2.dpad_left;
        boolean dpadRight = gamepad2.dpad_right;
        boolean buttonA = gamepad2.a;
        boolean buttonB = gamepad2.b;

        double rightTurn = 0;
        double leftTurn = 0;
        double resetIMU = 0;

        // changes power based on direction of turn to turn robot instead of strafe
        if (start) {
            resetIMU = hraezlyr.resetIMU();
        }
        // angle of controller stick

        double angle = Math.toDegrees(Math.atan2(leftY, leftX));
        angle = angle - hraezlyr.getHeading() + resetIMU;
        //angle = constrainAngle(angle);
        // scope orientation

        double power = Math.max(-1, Math.min(1, Math.sqrt((leftX * leftX) + (leftY * leftY))));

        // topLeftPower and bottomRightPower
        double powerGroup1 = ((Math.sin(Math.toRadians(angle))) - (Math.cos(Math.toRadians(angle)))) * power;
        // topRightPower and bottomLeftPower
        double powerGroup2 = ((Math.sin(Math.toRadians(angle))) + (Math.cos(Math.toRadians(angle)))) * power;




        // Power for drivetrain
        hraezlyr.topLeft.setPower(powerGroup1 - rightX);
        hraezlyr.topRight.setPower(powerGroup2 + rightX);
        hraezlyr.bottomLeft.setPower(powerGroup2 - rightX);
        hraezlyr.bottomRight.setPower(powerGroup1 + rightX);

        telemetry.addData("DpadUp", DpadUp);
        telemetry.addData("DpadDown", DpadDown);
        telemetry.addData("Angle", angle);
        telemetry.addData("Distance", power);
        telemetry.addData("rightX", rightX);
        telemetry.addData( "leftTurn", leftTurn);
        telemetry.addData("rightTurn", rightTurn);
        telemetry.addData("buttonA", buttonA);
        telemetry.addData("buttonB", buttonB);

        //telemetry.addData("Cascade Height", hraezlyr.cascadeMotor1.getCurrentPosition());
        telemetry.update();

        //System for cascade level system

        if(dpadRight){
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
        if(dpadLeft){
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

            if(DpadUp && !DpadDown) {
                hraezlyr.cascadeMotor1.setPower(0.3);
                hraezlyr.cascadeMotor2.setPower(0.3);
            }
            if(DpadDown && !DpadUp) {
                hraezlyr.cascadeMotor1.setPower(-0.3);
                hraezlyr.cascadeMotor2.setPower(-0.3);
            }
            if(!DpadDown && !DpadUp){
                hraezlyr.cascadeMotor1.setPower(0);
                hraezlyr.cascadeMotor2.setPower(0);
            }





        if(buttonA){

            hraezlyr.servoClawClose.setPosition(1);

        }

        if(buttonB) {

            hraezlyr.servoClawClose.setPosition(-1);
        }
    }



}
