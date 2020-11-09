package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;

@Autonomous
//@Disabled
public class TensorFlowTest extends LinearOpMode {
    Robot robot = new Robot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initVuforia();
        robot.initTfod();
        robot.initChassis(this);

        waitForStart();


    }
}
