package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
public class ClearLogs extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LoggingUtil.clearLogs(hardwareMap.appContext);
    }
}
