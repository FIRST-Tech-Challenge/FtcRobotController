package org.firstinspires.ftc.masters.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.RobotClass;

@Autonomous(name="test servo")
public class TestMovement extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap,telemetry,this);

        waitForStart();

        robot.linearSlideServo.setPosition(0.65);
        sleep(5000);

    }
}
