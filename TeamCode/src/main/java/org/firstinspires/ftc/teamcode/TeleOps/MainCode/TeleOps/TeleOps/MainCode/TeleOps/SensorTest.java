package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SensorTest extends LinearOpMode {
    private DistanceSensor sensor;
    private Servo s1;
    private double distance = 5;
    
    
    @Override
    public void runOpMode() throws InterruptedException {
        Gamepad g1 = new Gamepad();

        sensor = hardwareMap.get(DistanceSensor.class, "Sensor");
        s1 = hardwareMap.servo.get("s1");

        waitForStart();

        while(opModeIsActive()) {
            if (sensor.getDistance(DistanceUnit.CM) < distance) {
                s1.setPosition(.1);
            }
            telemetry.addLine("" + sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}