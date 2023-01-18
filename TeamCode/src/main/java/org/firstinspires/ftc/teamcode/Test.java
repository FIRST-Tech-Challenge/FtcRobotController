package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        DigitalChannel armSensor;


        armSensor = hardwareMap.get(DigitalChannel.class, "armSensor");
        armSensor.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        if (isStopRequested()) return;
        resetRuntime();
        while (opModeIsActive()){
            telemetry.addData("s", armSensor.getState());
            telemetry.update();
        }
//        Servo grabberRight = hardwareMap.servo.get("grabberRight");
//        Servo grabberLeft = hardwareMap.servo.get("grabberLeft");
//
//
//        grabberLeft.setDirection(Servo.Direction.REVERSE);
//
//        waitForStart();
//        if (isStopRequested()) return;
//        resetRuntime();
//
//        while (opModeIsActive()) {
//            grabberRight.setPosition(0.75 - gamepad1.left_stick_y / 25);
//            grabberLeft.setPosition(0.75 - gamepad1.left_stick_y / 25);
//
//            telemetry.addData("position", 0.75 - gamepad1.left_stick_y / 25);
//            telemetry.update();
//        }

    }
}
