package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Date;
import java.util.List;

@Config
@Autonomous(name = "Center Stage Backdrop Blue Pre Odo", group = "competition")
public class CenterStageBackdropBluePreOdo extends LinearOpMode {
//    enum State {
//        PURPLE_DEPOSIT_PATH,
//        PURPLE_DEPOSIT,
//
//        BACKUP_FROM_SPIKES,
//
//        YELLOW_DEPOSIT_PATH,
//        YELLOW_DEPOSIT,
//
//        PARK
//    }

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        robot = new Robot(hardwareMap, telemetry, this);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CenterStageComputerVisionPipelines CV = new CenterStageComputerVisionPipelines(hardwareMap, telemetry);
        CenterStageComputerVisionPipelines.pos propPos = null;

        waitForStart();

        long startTime = new Date().getTime();
        long time = 0;

        while (time < 50 && opModeIsActive()) {
            time = new Date().getTime() - startTime;
            propPos = CV.propFind.position;
            telemetry.addData("Position", propPos);
        }

        robot.forward(24.5674, .65);

    }
}