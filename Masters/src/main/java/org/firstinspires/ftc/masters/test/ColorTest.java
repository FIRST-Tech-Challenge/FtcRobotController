package org.firstinspires.ftc.masters.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("raw", color.rawOptical());
            telemetry.addData("red", color.red());
            telemetry.addData("blue", color.blue());
            telemetry.addData("green", color.green());
        }
    }
}
