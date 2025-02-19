package org.firstinspires.ftc.masters.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Color Test", group = "Test")
public class ColorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        RevColorSensorV3 color = hardwareMap.get(RevColorSensorV3.class, "color");

        waitForStart();

        while(opModeIsActive()){

            if (color.red()>color.blue() && color.red()> color.green()){
                telemetry.addData("Color", "Red");
            }
            else if (color.green()>color.blue() && color.green()>color.red()){
                telemetry.addData("Color", "yellow");
            } else if (color.blue()>color.green() && color.blue()>color.red()){
                telemetry.addData("Color", "Blue");
            }

            telemetry.addData("raw", color.rawOptical());
            telemetry.addData("red", color.red());
            telemetry.addData("blue", color.blue());
            telemetry.addData("green", color.green());
            telemetry.update();
        }
    }
}
