package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;

@Autonomous(name= "TensorFlowTest")
//@Disabled
public class TensorFlowTest extends LinearOpMode {
    Robot robot = new Robot();
    TensorFlow tensorFlow = new TensorFlow(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initChassis(this);
        robot.runTensorFlow();
        robot.initTensorFlow();
        robot.stopTensorFlow();
        waitForStart();

        tensorFlow.getNumberOfRings();
        telemetry.addData("Number of Rings", tensorFlow.getNumberOfRings());
        telemetry.update();
        sleep(5000);
        robot.stopTensorFlow();
    }
}
