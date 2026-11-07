package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Robot;

public class TestDriveOpMode extends LinearOpMode {

    public Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            //Put all gamepad logic here.
            robot.drive.setDriverInput(-gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x);
            if (gamepad1.aWasPressed()){
                robot.drive.toggleMode();
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());

            robot.periodic();
            robot.writeTelemetry(telemetry);
        }

    }
}
