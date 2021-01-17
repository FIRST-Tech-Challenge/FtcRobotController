package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.competition.Hardware;

@TeleOp(name = "ring test ", group="test")
public class TestRingFire extends OpMode
{


    Hardware robot;
    double servoPosition=1;
    double angleSpeed = .2;
    boolean upPressed;
    boolean downPressed;
    boolean flickPress = false;

    @Override
    public void init()
    {

        robot = new Hardware();
        robot.init(hardwareMap);

    }

    @Override
    public void loop()
    {

        if(Math.abs(gamepad1.left_stick_y)>.03)
            servoPosition-=gamepad1.left_stick_y*angleSpeed/20;

        if(servoPosition>1)
            servoPosition=1;
        else if(servoPosition<.17)
            servoPosition=.17;
        if(gamepad1.dpad_up&&!upPressed)
        {
            if(angleSpeed<1)
                angleSpeed+=.1;
            upPressed=true;
        }
        if(!gamepad1.dpad_up)
        {

            upPressed=false;

        }

        if(gamepad1.dpad_down&&!downPressed)
        {
            if(angleSpeed>.11)
                angleSpeed-=.1;
            downPressed=true;
        }



        robot.flywheelRotateServoLeft.setPosition(servoPosition);
        if(gamepad1.a)
            robot.setFlyWheelVelocity(1126);
        else if(gamepad1.b)
            robot.setFlyWheelVelocity(1126*2);
        else if(gamepad1.x)
            robot.setFlyWheelVelocity(1126*2.1);
        else if(gamepad1.y)
            robot.setFlyWheelVelocity(4*1126);
        else if(gamepad1.left_bumper)
            robot.setFlyWheelPower(1);
        else
            robot.setFlyWheelPower(0);
        if(gamepad1.right_bumper)
        {
            if (!flickPress)
            {
                robot.flickRing();
                flickPress=true;
            }
        }
        else
            flickPress=false;
        telemetry.addData("Left Fly Wheel Velocity", robot.flywheelMotorLeft.getVelocity());
        telemetry.addData("Right Fly Wheel Velocity", robot.flywheelMotorRight.getVelocity());

        telemetry.update();

    }

}
