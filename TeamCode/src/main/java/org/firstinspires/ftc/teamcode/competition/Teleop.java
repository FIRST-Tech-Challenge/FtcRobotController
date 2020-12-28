package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

@TeleOp(name = "Teleop", group = "DemoBot")
public class Teleop extends LinearOpMode
{


    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        waitForStart();
        boolean a2Pressed=false;
        boolean a1Pressed = false;
        boolean y2Pressed=false;
        boolean upPressed=false;
        boolean downPressed=false;
        boolean autoAim=false;
        double forward=1;
        double angleSpeed=1;
        while(opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            if(autoAim)
            {

                byte sign = 1;
                double diff = robot.theta-Math.atan((36-robot.x)/-robot.y);
                if(diff<0)
                    diff+=2*Math.PI;
                if(diff>Math.PI)
                    sign=-1;
                if(diff>Math.PI)
                    diff=Math.PI*2-diff;
                robot.drive(forward*gamepad1.left_stick_y,gamepad1.left_stick_x, HelperMethods.clamp(-1,sign*diff,1));

            }
            else
                robot.drive(forward*gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.addData("left encoder",robot.leftEncoderPos);
            telemetry.addData("right encoder",robot.rightEncoderPos);
            telemetry.addData("center encoder",robot.centerEncoderPos);
            telemetry.addData("angle speed",angleSpeed);
            telemetry.addData("Left Speed", robot.flywheelMotorLeft.getVelocity());
            telemetry.addData("Right Speed", robot.flywheelMotorRight.getVelocity());
            telemetry.update();

            if(gamepad1.a&&!a1Pressed)
            {
                autoAim=!autoAim;
                a1Pressed=true;
            }
            if(!gamepad1.a)
            {

                a1Pressed=false;

            }

            if(gamepad1.dpad_up)
            {

                forward=1;

            }
            if(gamepad1.dpad_down)
            {

                forward=-1;

            }

            if (gamepad1.right_trigger<.01) {


                robot.setIntakePower(-gamepad1.left_trigger);

            }
            else
            {

                robot.setIntakePower(gamepad1.right_trigger);

            }


            //Setting power for intake to the right trigger



            //sets flywheel power to the left trigger
            robot.setFlyWheelPower(gamepad2.right_trigger);

            //makes the flywheel rotation servo move with b and x
            if(gamepad2.right_bumper)
            {

                robot.flywheelRotateServoLeft.setPower(angleSpeed);

            }

            else if(gamepad2.left_bumper)
            {

                robot.flywheelRotateServoLeft.setPower(-angleSpeed);

            }

            else
            {

                robot.flywheelRotateServoLeft.setPower(0);

            }

            //launch a single ring if a is pressed and the flywheels are moving
            if(gamepad2.a&&!a2Pressed&&(Math.abs(robot.flywheelMotorLeft.getVelocity())>20||gamepad2.x))
            {
                robot.flickRing();
                a2Pressed=true;
            }
            if(!gamepad2.a)
            {

                a2Pressed=false;

            }

            //launch 3 rings if y is pressed and flywheels are moving
            if(gamepad2.y&&!y2Pressed&&(Math.abs(robot.flywheelMotorLeft.getVelocity())>20||gamepad2.x))
            {
                robot.queuedFlicks=2;
                robot.flickRing();
                y2Pressed=true;
            }
            if(!gamepad2.y)
            {

                y2Pressed=false;

            }

            if(gamepad2.dpad_up&&!upPressed)
            {
                if(angleSpeed<1)
                    angleSpeed+=.1;
                upPressed=true;
            }
            if(!gamepad2.dpad_up)
            {

                upPressed=false;

            }

            if(gamepad2.dpad_down&&!downPressed)
            {
                if(angleSpeed>.11)
                    angleSpeed-=.1;
                downPressed=true;
            }
            if(!gamepad2.dpad_down)
            {

                downPressed=false;

            }

        }

        ThreadPool.renewPool();

    }

}