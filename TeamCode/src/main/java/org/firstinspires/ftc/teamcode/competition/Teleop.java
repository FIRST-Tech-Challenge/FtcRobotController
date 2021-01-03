package org.firstinspires.ftc.teamcode.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.helperclasses.HelperMethods;
import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

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
        boolean a1Pressed = true;
        boolean y2Pressed=false;
        boolean b1Pressed = false;
        boolean upPressed=false;
        boolean downPressed=false;
        boolean autoAim=false;
        boolean slowDrive = false;
        double forward=1;
        double angleSpeed=.2;
        double servoPosition =1;
        double savedPosition = .41;

        double leftSub = 0;
        double rightSub = 0;
        double centerSub=0;

        double driveSpeed=1;
        while(opModeIsActive())
        {

            robot.updatePositionRoadRunner();
            //set drive speed
            if(slowDrive)
                driveSpeed=.3;
            else
                driveSpeed=1;

            //automatically aim  at goal
            if(autoAim)
            {

                byte sign = 1;
                double diff = robot.theta-Math.atan((50+robot.y)/(9-robot.x))*.8;
                if(diff<0)
                    diff+=2*Math.PI;
                if(diff>Math.PI)
                    sign=-1;
                if(diff>Math.PI)
                    diff=Math.PI*2-diff;
                if(Math.abs(diff)>.04&&Math.abs(diff)<.2)
                    diff=.2*Math.abs(diff)/diff;
                else if(Math.abs(diff)<.04)
                    diff*=4;
                robot.drive(forward*driveSpeed*gamepad1.left_stick_y,driveSpeed*gamepad1.left_stick_x, HelperMethods.clamp(-.9,sign*diff,.9));

            }
            else
                robot.drive(forward*driveSpeed*gamepad1.left_stick_y,driveSpeed*gamepad1.left_stick_x,driveSpeed*gamepad1.right_stick_x);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.addData("Auto Aim",autoAim);
            telemetry.addData("Slow Drive",slowDrive);
            telemetry.addData("angle speed",angleSpeed);
            telemetry.addData("odom left",(robot.odom.getWheelPositions().get(1)-leftSub)*2048*4/0.688975/Math.PI/2);
            telemetry.addData("odom right",(robot.odom.getWheelPositions().get(2)-rightSub)*2048*4/0.688975/Math.PI/2);
            telemetry.addData("odom center",(robot.odom.getWheelPositions().get(0)-centerSub)*2048*4/0.688975/Math.PI/2);
            telemetry.addData("Left Speed", robot.flywheelMotorLeft.getVelocity());
            telemetry.addData("Right Speed", robot.flywheelMotorRight.getVelocity());
            telemetry.addData("Angle Servo",servoPosition);
            telemetry.update();

            if(gamepad1.x)
            {

                leftSub=robot.odom.getWheelPositions().get(1);
                rightSub = robot.odom.getWheelPositions().get(2);
                centerSub = robot.odom.getWheelPositions().get(0);

            }

            if(gamepad1.a&&!a1Pressed)
            {
                autoAim=!autoAim;
                a1Pressed=true;
            }
            if(!gamepad1.a)
            {

                a1Pressed=false;

            }
            if(gamepad1.b&&!b1Pressed&&!gamepad1.start)
            {
                slowDrive=!slowDrive;
                b1Pressed=true;
            }
            if(!gamepad1.b)
            {

                b1Pressed=false;

            }

            //flip direction of drive when dpad buttons are pressed
            if(gamepad1.dpad_up)
            {

                forward=1;

            }
            if(gamepad1.dpad_down)
            {

                forward=-1;

            }

            //intake or outtake rings
            if (gamepad1.right_trigger<.01)
            {


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
            if(Math.abs(gamepad2.left_stick_y)>.03)
            servoPosition+=gamepad2.left_stick_y*angleSpeed/20;

            if(servoPosition>1)
                servoPosition=1;
            else if(servoPosition<.17)
                servoPosition=.17;



            robot.flywheelRotateServoLeft.setPosition(servoPosition);



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
            if(gamepad2.left_trigger>.01)
            {

                savedPosition=servoPosition;

            }

            if(gamepad2.b)
            {


                servoPosition=savedPosition;

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