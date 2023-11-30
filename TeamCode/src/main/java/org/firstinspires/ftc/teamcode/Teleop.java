package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "A Teleop", group = "Teleop")
public class Teleop extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        Hardware robot = new Hardware(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {

            //Drivetrain code
            robot.robotODrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            //Transfer Code --
            if (gamepad2.left_stick_y > .05){
                robot.TransferM1.setPower(1);
                robot.TransferM2.setPower(1);
            } else {
                robot.TransferM1.setPower(0);
                robot.TransferM2.setPower(0);
            }
            // -- End Transfer code

            //Intake code --
            //Spin Inwards
            if(gamepad1.right_trigger > .05) {
                robot.intakeMotor.setPower(gamepad1.right_trigger);
                robot.counteroller.setPower(-gamepad1.right_trigger);
                robot.TransferCR1.setPower(-1);
                robot.TransferCR2.setPower(1);
            }
            //Spin Outwards
            else if(gamepad1.left_trigger > .05) {
                robot.intakeMotor.setPower(-gamepad1.right_trigger);
                robot.counteroller.setPower(gamepad1.right_trigger);
                robot.TransferCR1.setPower(1);
                robot.TransferCR2.setPower(-1);
            }
            //If neither are pressed or both are pressed everything is set to it's zeroPowerBehavior()
            else if((gamepad1.left_trigger > .05  && gamepad1.right_trigger > .05) || (gamepad1.left_trigger < .05 && gamepad1.right_trigger < .05)) {
                robot.intakeMotor.setPower(0);
                robot.counteroller.setPower(0);
                robot.TransferCR1.setPower(0);
                robot.TransferCR2.setPower(0);
            }
            // -- End Intake Code


            //Deposit code --
            if (gamepad2.right_trigger > 0.05)
                robot.depositServoOne.setPosition(1);
             else
                robot.depositServoOne.setPosition(0);
            if (gamepad2.left_trigger > 0.05)
                robot.depositServoTwo.setPosition(1);
             else
                robot.depositServoTwo.setPosition(0);
            // -- End Deposit Code
        }}}