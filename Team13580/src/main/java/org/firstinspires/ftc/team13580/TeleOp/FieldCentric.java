package org.firstinspires.ftc.team13580.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.team13580.RobotHardware;

@TeleOp (name = "Field Centric", group = "Robot")

public class FieldCentric extends LinearOpMode {
    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double axial = 0;
        double lateral = 0;
        double yaw = 0;
        double handOffset = 0;
        double upDown = 0;
        double spoolie = 0;
        double armPositionFudgeFactor;
        double hangPositionFudgeFactor;
        double hang=0;
        double elbowHang=0;


        robot.init();
        double armPosition = robot.ARM_COLLAPSED_INTO_ROBOT;
        double spooliePosition= robot.SPOOLIE_COLLAPSED;
        boolean claw=false;

        int clawTimer = 0;

        waitForStart();

        while (opModeIsActive()) {
            //Assigned the movement to the joystick in the game controller
            axial = -gamepad1.left_stick_y ;
            lateral = gamepad1.left_stick_x * 1.1;
            yaw = gamepad1.right_stick_x;
            //passes the values to the method called driveFieldCentric in the robot hardware class
            robot.driveFieldCentric(axial, lateral, yaw);

            // assigns the gamepad 1 right bumper to open the claw  and the gamepad 1 left bumper to close the claw


            if(gamepad2.right_stick_button && clawTimer >= 13){
               claw=!claw;
               clawTimer = 0;
           }
            clawTimer++;

            if (claw) {
                robot.leftHand.setPosition(0);

                //handOffset += robot.HAND_SPEED;
            } else if(!claw){
                //handOffset -= robot.HAND_SPEED;
                robot.leftHand.setPosition(0.35);
            }


            //handOffset = Range.clip(handOffset, -0.1, 0.5);
            //handOffset = Range.clip(handOffset,0.05,-0.1);
            //passes the positions of the hand to the robotHardware class without this line it will not move
            //robot.setHandPositions(handOffset);

            //passes the position of the elbow of the robot(the motor in the arm, not the spoolie)
            //to up when pressing the right bumper in the game controller 2
            // ad to down when pressing the left bumper of gam controller 2
            if (gamepad2.right_bumper) {
                spoolie = robot.SPOOLIE_UP_POWER;
            } else if (gamepad2.left_bumper) {
                spoolie = robot.SPOOLIE_DOWN_POWER;
            } else {
                spoolie = 0;
            }
            robot.setSpooliePower(spoolie);

            if (gamepad1.x) {
                hang = robot.ARM_UP_POWER;
            } else if (gamepad1.y) {
                hang = robot.ARM_DOWN_POWER;
            } else {
                hang = 0;
            }
            robot.setHangPower(hang);

            //position of the arm of the robot using encoders
            if (gamepad2.a) {
                armPosition = robot.ARM_COLLAPSED_INTO_ROBOT;
            } else if (gamepad2.b) {
                armPosition = robot.ARM_ATTACH_HANGING_HOOK;
            } else if (gamepad2.x) {
                //robot.wrist.setPosition(1);
                armPosition = robot.ARM_CLEAR_BARRIER;
            } else if (gamepad2.y) {
                armPosition = robot.ARM_SCORE_SAMPLE_IN_LOW;
            }else if (gamepad2.dpad_down){
                //robot.setSpooliePower(100);
                //robot.wrist.setPosition(0);
                armPosition= robot.ARM_SECURE_SPECIMEN;
            }else if(gamepad2.dpad_up){
                armPosition = robot.ARM_SCORE_SPECIMEN;
            }else if(gamepad2.dpad_left){
               armPosition= robot.ARM_SCORE;
            }else if(gamepad2.dpad_right){
                armPosition= robot.ARM_SPECIMEN;
            }else if(gamepad2.left_stick_button){
                armPosition=robot.ARM_COLLECT;
            }
            armPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad2.right_trigger + (-gamepad2.left_trigger));

            if(gamepad1.a){
                robot.wrist.setPosition(0.04);
            }else if(gamepad1.b){
                robot.wrist.setPosition(.70);
            }
            robot.upDown.setTargetPosition((int) (armPosition + armPositionFudgeFactor));
            ((DcMotorEx)robot.upDown).setVelocity(2100);
            robot.upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(gamepad1.a){
                elbowHang= robot.HANG_UP;
            }else if(gamepad1.b){
                elbowHang= robot.HANG_COLLAPSED_INTO_ROBOT;
            }
            hangPositionFudgeFactor = robot.FUDGE_FACTOR * (gamepad1.right_trigger + (-gamepad1.left_trigger));

            robot.elbowHang.setTargetPosition((int) (elbowHang + hangPositionFudgeFactor));
            ((DcMotorEx)robot.elbowHang).setVelocity(1000);
            robot.elbowHang.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        }
    }
}