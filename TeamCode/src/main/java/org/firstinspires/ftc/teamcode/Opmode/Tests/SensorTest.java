package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
@Disabled

public class SensorTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        DigitalChannel pin0 = hardwareMap.digitalChannel.get("color0");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("color1");

        waitForStart();
        //yellow: 0-true, 1-true
        //blue: 0-true 1-false
        //red: 0-false 1-true
        while (opModeIsActive()) {
            if(pin0.getState() && pin1.getState()){
                telemetry.addData("Color: ", "Yellow");
            }else if(pin0.getState() && !pin1.getState()){
                telemetry.addData("Color: ", "Blue");
            }else if(!pin0.getState() && pin1.getState()){
                telemetry.addData("Color: ", "Red");
            }else{
                telemetry.addData("Color: ", "Null");
            }
            telemetry.update();
        }
    }
}
