package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@Disabled
@TeleOp
public class LimitSwitchTest extends LinearOpMode {

    @Override
    public void runOpMode(){
        DigitalChannel digitalChannel = hardwareMap.get(DigitalChannel.class , "LS");
        digitalChannel.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("state", digitalChannel.getState());
            telemetry.update();
        }

    }
}
