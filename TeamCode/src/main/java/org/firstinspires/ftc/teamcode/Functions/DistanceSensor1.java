package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name = "REV2mDistance", group = "TEST")
@Disabled
public class DistanceSensor1 extends OpMode {

    private DistanceSensor sensorRange;



    @Override
    public void init()
    {
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
    }
    public void distance2()
    {
        double value = sensorRange.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", value);
    }

    public void distance()
    {
        double value2 = 0;

        for (int i = 0; i < 10; i++) {
            double value = sensorRange.getDistance(DistanceUnit.CM);

            if (value < 819)
                value2 += value;
        }

        value2 /= 10;

        telemetry.addData("Distanta: ", value2);
    }

    @Override
    public void loop()
    {
        distance();
    }

    @Override
    public void stop(){

    }
}
