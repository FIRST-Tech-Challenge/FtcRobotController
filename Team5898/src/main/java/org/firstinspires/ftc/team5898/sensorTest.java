package org.firstinspires.ftc.team5898;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor Test",group = "Test")
public class sensorTest extends LinearOpMode {
    ColorSensor color;
    DistanceSensor distance;

    @Override
    public void runOpMode(){
        color = hardwareMap.get(ColorSensor.class,"color");
        distance = hardwareMap.get(DistanceSensor.class,"distance");

        waitForStart();

        while(opModeIsActive()){
            telemetry.addData("Light Detected",((OpticalDistanceSensor) color).getLightDetected());
            telemetry.addData("Red",color.red());
            telemetry.addData("Blue",color.blue());
            telemetry.addData("Green",color.green());
            telemetry.addData("Distance from the object",distance.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

}
