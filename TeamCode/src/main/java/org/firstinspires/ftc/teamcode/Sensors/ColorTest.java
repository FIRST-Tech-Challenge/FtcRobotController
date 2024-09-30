package org.firstinspires.ftc.teamcode.hardware.Sensors;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorTest extends LinearOpMode{
    @Override
    public void runOpMode() {

        ColorSensor pixel1 = hardwareMap.get(RevColorSensorV3.class, "color1");
        waitForStart();
        while(opModeIsActive()){

            telemetry.addData("Color", findColor(pixel1.blue(), pixel1.red(), pixel1.green()));
            telemetry.update();
        }
    }

    public String findColor(int blue, int red, int green){
        double ratiorb = (double)red/blue;
        double ratiogb = (double)green/blue;

        double tolerance = 0.2;
        telemetry.addData("red/blue", ratiorb);
        telemetry.addData("green/blue", ratiogb);
        if(Math.abs(ratiorb-2.672)<=tolerance && Math.abs(ratiogb-3.78)<=tolerance){
            return "YELLOW";
        }
        if(Math.abs(ratiorb-0.5)<=tolerance && Math.abs(ratiogb-0.7)<=tolerance){
            return "PURPLE";
        }
        if(Math.abs(ratiorb-0.7)<=tolerance && Math.abs(ratiogb-2.5)<=tolerance){
            return "GREEN";
        }
        if(Math.abs(ratiorb-0.66)<=tolerance && Math.abs(ratiogb-1.2)<=tolerance){
            return "WHITE";
        }
        return "NOTHING";
    }
}
