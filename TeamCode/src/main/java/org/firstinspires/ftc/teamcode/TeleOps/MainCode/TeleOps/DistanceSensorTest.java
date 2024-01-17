package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    private DistanceSensor sensor;
    private Servo s1;
    private double distance = 3;  // Set the maximum distance in CM for servo activation

    @Override
    public void runOpMode() throws InterruptedException {
        sensor = hardwareMap.get(DistanceSensor.class, "DS1");
        s1 = hardwareMap.servo.get("s1");

        waitForStart();

        while(opModeIsActive()) {
            if (sensor.getDistance(DistanceUnit.CM) < distance) {
                s1.setPosition(.1);
//                s2.setPosition(.1);
            }
            telemetry.addData("Distance detected: ", sensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
