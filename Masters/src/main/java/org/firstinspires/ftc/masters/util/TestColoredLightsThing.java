package org.firstinspires.ftc.masters.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.RobotClass;

@Disabled
//@Autonomous(name = "Test Color Lights Blink A Lot Thing",group= "test")
public class TestColoredLightsThing extends LinearOpMode {
    RobotClass robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotClass(hardwareMap, telemetry, this);
        robot.lightSet();

        waitForStart();



        robot.greenLED.setState(true);
        robot.redLED.setState(true);
        robot.pauseButInSecondsForThePlebeians(2);
        robot.greenLED.setState(false);
        robot.redLED.setState(false);

        robot.getCube();

    }
}
