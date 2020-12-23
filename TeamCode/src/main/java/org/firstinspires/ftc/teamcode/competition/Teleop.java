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
            robot.drive(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            telemetry.addData("x: ", robot.x);
            telemetry.addData("y: ", robot.y);
            telemetry.addData("theta: ", robot.theta);
            telemetry.update();

        }

    }

}
