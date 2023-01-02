package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "SkystoneDetectionTest")
//@Disabled
public class SkystoneDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        SkystoneDetectionMaster cv = new SkystoneDetectionMaster(hardwareMap);
//      call the function to startStreaming
        cv.observeStone();
        waitForStart();
        while (opModeIsActive()) {
        }
//        stopStreaming
        cv.stopCamera();
    }
}
