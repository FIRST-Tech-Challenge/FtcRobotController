package org.firstinspires.ftc.team6220_PowerPlay;

abstract public class BaseTeleOp extends BaseOpMode{
    public void teleOpDrive(){
        if(Math.abs(Math.atan2(gamepad1.left_stick_y, gamepad2.left_stick_x)) > 5){
            driveRobot(gamepad1.left_stick_x, 0, gamepad1.right_stick_x);
        }else if(Math.abs(Math.atan2(gamepad1.left_stick_x, gamepad2.left_stick_y)) > 5){
            driveRobot(0, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }else{
            driveRobot(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
};