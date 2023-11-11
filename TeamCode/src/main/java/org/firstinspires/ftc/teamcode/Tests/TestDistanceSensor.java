package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class TestDistanceSensor extends LinearOpMode {
    DistanceSensor distance;
    DcMotor motor;

    @Override
    public void runOpMode() {
        //Get the distance sensor and motor from hardwareMap
        distance = hardwareMap.get(DistanceSensor.class, "Distance");
        motor = hardwareMap.get(DcMotor.class, "Motor");

        //Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive());
            //If the distance in centimeters is less than 10, set the power to 0
            if (distance.getDistance(DistanceUnit.CM) < 10) {
            motor.setPower(0.3);
            }
        else {
            motor.setPower(1);
        }
    }
}
