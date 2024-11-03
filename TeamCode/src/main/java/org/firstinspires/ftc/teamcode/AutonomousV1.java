package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "AutoTestBlue", group = "Auto")
public class AutonomousV1 extends LinearOpMode {
    protected OdometryBot odometryBot = new OdometryBot(this);
    @Override
    public void runOpMode() throws InterruptedException {

        odometryBot.isAuto = true;
        odometryBot.init(hardwareMap);



        while (!opModeIsActive()) {
        }

        waitForStart();


        telemetry.update();

        odometryBot.driveToCoordinate(0, 0, 0, 1000, 0.5, false);
        odometryBot.waitForCoordinateDrive();

    }
}