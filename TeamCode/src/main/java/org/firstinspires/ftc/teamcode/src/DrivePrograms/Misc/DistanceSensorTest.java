package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DistanceSensorTest")

public class DistanceSensorTest extends LinearOpMode {
    DcMotor intake;
    DistanceSensor distanceSensor;
    Servo bucketServo;


    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.dcMotor.get("intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bucketServo = hardwareMap.servo.get("bucket");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {

            intake.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
            telemetry.addData("distance:", distanceSensor.getDistance(DistanceUnit.CM));

            if (distanceSensor.getDistance(DistanceUnit.CM) < 9) {
                telemetry.addData("item detected", "");
            }


            telemetry.update();

        }

    }
}
