package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.ObjectDetection.TensorFlow;

@Autonomous
//@Disabled
public class TensorFlowTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Robot robot=new Robot(this, BasicChassis.ChassisType.ENCODER);
        TensorFlow tensorFlow = new TensorFlow(this);
        ElapsedTime runtime = new ElapsedTime();

        robot.initTensorFlow();
        robot.runTensorFlow();
        int rings = robot.tensorFlow.getNumberOfRings();
        telemetry.addData("Nummber of Rings: ", rings);
        telemetry.update();
        sleep(2000);
        robot.stopTensorFlow();

        waitForStart();

        if (rings == 4) {
            robot.moveForward(10,0.5); //moves forward if tensorflow detects 4 rings
        } else if (rings == 1) {
            robot.moveLeft(10,0.5); //moves left if tensorflow detects 1 ring
        } else {
            robot.moveRight(10, 0.5); //moves right if tensorflow detects 0 rings
        }
    }
}
