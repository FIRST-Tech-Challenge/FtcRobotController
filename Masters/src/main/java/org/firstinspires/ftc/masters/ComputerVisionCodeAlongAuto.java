package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;

@Autonomous(name="Computer Vision Test")
public class ComputerVisionCodeAlongAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        ComputerVisionCodeAlong CV = new ComputerVisionCodeAlong(hardwareMap, telemetry);

        ComputerVisionCodeAlong.ElementDetectionPipeline.ElementPosition itemLocation = null;

        itemLocation = CV.pipeline.position;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(CV.webcam, 10);

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 200 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            itemLocation = CV.pipeline.position;

            telemetry.addData("Position", itemLocation);
            telemetry.addData("Analysis", ComputerVisionCodeAlong.ElementDetectionPipeline.avg);
            telemetry.update();
        }

        switch (itemLocation) {
            case PRESENT:

                break;
            case NOT_PRESENT:

                break;
            default:


            }

    }



}
