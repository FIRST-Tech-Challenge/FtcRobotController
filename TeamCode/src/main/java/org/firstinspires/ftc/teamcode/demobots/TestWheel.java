package org.firstinspires.ftc.teamcode.demobots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.competition.Hardware;

@TeleOp(name="test", group="test")
public class TestWheel extends LinearOpMode
{
    @Override
    public void runOpMode() throws InterruptedException
    {

        Hardware robot = new Hardware();
        robot.init(hardwareMap);
        waitForStart();
        while(opModeIsActive())
        {
            if (gamepad1.a)
                robot.leftFront.setPower(1);
            else
                robot.leftFront.setPower(0);
            if (gamepad1.b)
                robot.rightFront.setPower(1);
            else
                robot.rightFront.setPower(0);
            if (gamepad1.y)
                robot.leftRear.setPower(1);
            else
                robot.leftRear.setPower(0);
            if (gamepad1.x)
                robot.rightRear.setPower(1);
            else
                robot.rightRear.setPower(0);

        }
    }
}
