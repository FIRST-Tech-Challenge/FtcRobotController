package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;

@Autonomous
@Disabled
public class TensorFlowTest extends LinearOpMode {
    Robot robot = new Robot();
    TensorFlow tensorFlow = new TensorFlow(this);
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.initTensorFlow();
        robot.runTensorFlow();
        robot.initChassis(this);
        robot.stopTensorFlow();

        waitForStart();

        robot.runTensorFlow();
        if (tensorFlow.getNumberOfRings() == 4) {
            robot.moveForward(10,0.5);
        } else if (tensorFlow.getNumberOfRings() == 1) {
            robot.moveLeft(10,0.5);
        } else {
            robot.moveRight(10, 0.5);
        }
    }
}
