package org.firstinspires.ftc.masters.apriltesting.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Config
@Autonomous(name = "Limit Switch Test", group="pid")
public class LimitSwitchTest extends LinearOpMode {

    private DigitalChannel digIn;

    @Override
    public void runOpMode() {

        digIn = hardwareMap.digitalChannel.get("limSwitch");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Is pressed? ",digIn.getState());
            telemetry.update();
        }

    }




}
