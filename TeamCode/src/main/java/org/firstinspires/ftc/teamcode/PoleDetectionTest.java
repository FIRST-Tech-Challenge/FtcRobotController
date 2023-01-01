package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous(name = "PoleDetectionTest")
//@Disabled
public class PoleDetectionTest extends LinearOpMode {

    @Override
    public void runOpMode() {
//        initialize camera and pipeline
        PoleDetectionMaster cv = new PoleDetectionMaster(hardwareMap);
//      call the function to startStreaming
        cv.observeStick();
        waitForStart();
        while (opModeIsActive()) {
        }
//        stopStreaming
        cv.stopCamera();
    }
}
