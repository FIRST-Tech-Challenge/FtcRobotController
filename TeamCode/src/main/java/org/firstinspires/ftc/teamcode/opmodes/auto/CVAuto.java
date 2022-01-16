package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
@Autonomous
public class CVAuto extends LinearOpMode {
    public EventThread eventThread = new EventThread();
    @Override
    public void runOpMode(){
        telemetry.addLine("Starting");
        telemetry.update();
        TseDetector webcam = new TseDetector(hardwareMap, "webcam", true);
        waitForStart();
        telemetry.addLine("Running");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("webcamOutput", webcam.run(true));
            telemetry.update();
        }
        eventThread.interrupt();
        requestOpModeStop();
    }
}
