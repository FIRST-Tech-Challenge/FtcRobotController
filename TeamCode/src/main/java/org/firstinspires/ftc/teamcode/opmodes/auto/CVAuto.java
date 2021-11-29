package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.robot.vision.robot.TseDetector;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;

@Autonomous
public class CVAuto extends LinearOpMode {
    public EventThread eventThread = new EventThread(this::opModeIsActive);
    @Override
    public void runOpMode(){
        TseDetector webcam = new TseDetector(eventThread, hardwareMap, "webcam");
        Thread thread = new Thread(() -> {
            telemetry.addData("webcamOutput", webcam.run());
            telemetry.update();
        });
        thread.setPriority(4);
        waitForStart();
        thread.start();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive()) { }
        requestOpModeStop();
    }
}
