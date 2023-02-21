package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;


@TeleOp(name="ColorTeleOP", group = "Sensor")
@Disabled
public class TeleOPColor extends OpMode {

    ColorSensor color;






    @Override
    public void init() {
        color = hardwareMap.get(ColorSensor.class, "sensor_color");



    }


    @Override
    public void loop() {

        telemetry.addData("Red", color.red());
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Green", color.green());
        telemetry.update();
    }
}
