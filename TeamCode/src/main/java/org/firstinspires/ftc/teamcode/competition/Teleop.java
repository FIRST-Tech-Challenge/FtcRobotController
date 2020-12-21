package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

@TeleOp(name = "Teleop", group = "DemoBot")
public class Teleop extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            robot.drive(gamepad1.right_stick_y,gamepad1.right_stick_x,gamepad1.left_stick_x);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.update();


            robot.setIntakePower(gamepad1.right_trigger);
            //Setting power for intake to the right trigger
             if (gamepad1.right_trigger<.01) {


                 robot.setIntakePower(-gamepad1.left_trigger);
             }


            //sets flywheel power to the left trigger
            robot.setFlyWheelPower(gamepad2.right_trigger);

            //makes the flywheel rotation servo move with b and x
            if(gamepad2.b){

                robot.flywheelRotateServoLeft.setPower(1);

            }

            else if(gamepad2.x){

                robot.flywheelRotateServoLeft.setPower(-1);

            }

            else {

                robot.flywheelRotateServoLeft.setPower(0);

            }

        }

    }

}