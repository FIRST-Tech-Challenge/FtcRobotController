package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Drive")
public class Drive extends Control {

    @Override
    public void init() {
        super.init();

        hraezlyr.cascadeMotor1.setPower(0);
        hraezlyr.cascadeMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hraezlyr.cascadeMotor2.setPower(0);
        hraezlyr.cascadeMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        boolean DpadPressed;
        double rightTurn = 0;
        double leftTurn = 0;
        double resetIMU = 0;

        // changes power based on direction of turn to turn robot instead of strafe
        if (rightX > 0) {
            rightTurn = rightX;
        }
        if (rightX < 0) {
            leftTurn = rightX;
        }

        if (start) {
            resetIMU = hraezlyr.resetIMU();
        }
        // angle of controller stick
        double angle = Math.atan2(leftY, leftX) - hraezlyr.getHeading() - resetIMU;
        // scope orientation
        // distance is the power of the controller stick
        double distance = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));

        // topLeftPower and bottomRightPower
        double powerGroup1 = (Math.sin(angle) - Math.cos(angle)) * distance;
        // topRightPower and bottomLeftPower
        double powerGroup2 = (Math.sin(angle) + Math.cos(angle)) * distance;

        // Power for drivetrain
        hraezlyr.topLeft.setPower(powerGroup1 - leftTurn + rightTurn);
        hraezlyr.topRight.setPower(powerGroup2 - rightTurn + leftTurn);
        hraezlyr.bottomLeft.setPower(powerGroup2 - rightTurn + leftTurn);
        hraezlyr.bottomRight.setPower(powerGroup1 - leftTurn + rightTurn);

        telemetry.addData("Drive power1", powerGroup1);
        telemetry.addData("Drive power2", powerGroup2);
        telemetry.addData("Cascade Height", hraezlyr.cascadeMotor1.getCurrentPosition());
        telemetry.update();

        //System for cascade level system

        if(gamepad2.dpad_up){
            switch(zHeight){
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
        if(gamepad2.dpad_down){
            switch(zHeight) {
                case HIGH:
                    zHeight = Level.MEDIUM;
                    break;
                case MEDIUM:
                    zHeight = Level.LOW;
                    break;
                case LOW:
                    zHeight = Level.GROUND;


            }
        }
    }
}