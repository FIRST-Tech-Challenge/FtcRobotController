package org.firstinspires.ftc.teamcode.opmodes.tests;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.vision.demo.pipeline1Detector;

public class Pipeline1Implementation extends LinearOpMode {
    pipeline1Detector pipeline1Detector;
    @Override
    public void runOpMode() {
        pipeline1Detector = new pipeline1Detector(hardwareMap, "webcam");
        pipeline1Detector.init();
        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // pipeline1Detector.Stack stack = pipeline1Detector.getStack();
            // telemetry.addData("Rings", stack);
            telemetry.update();
        }
    }
}
