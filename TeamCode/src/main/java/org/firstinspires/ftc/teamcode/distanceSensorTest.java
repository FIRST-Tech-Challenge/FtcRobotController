package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "distanceSensor", group = "Sensor")

public class distanceSensorTest extends OpMode {
    DistanceSensor dsensor;

    @Override
    public void init(){
        dsensor = hardwareMap.get(DistanceSensor.class, "distanceTest");
    }

    public void distance(){
        double value = dsensor.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance: ", value);
    }

    @Override
    public void loop(){
        distance();
    }
    @Override
    public void stop(){

    }
}
//not updating distance just outputing same value
