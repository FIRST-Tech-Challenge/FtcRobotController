package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot2.FiducialDetector;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

@Autonomous
@Config
public class FiducialAuto extends LinearOpMode {
    public static int zeroOrOneRedorBlue = 0;
    public EventThread eventThread = new EventThread();
    @Override
    public void runOpMode(){
        telemetry.addLine("Starting");
        telemetry.update();
        FiducialDetector webcam = new FiducialDetector(hardwareMap, "webcam", true, zeroOrOneRedorBlue == 0);
        waitForStart();
        telemetry.addLine("Running");
        telemetry.update();
        while (opModeIsActive()) {
            telemetry.addData("webcamOutput", webcam.run());
            telemetry.update();
        }
        eventThread.interrupt();
        requestOpModeStop();
    }
}
