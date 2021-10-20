package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ArmMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo motor = hardwareMap.get(Servo.class,"motor");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();
        while (opModeIsActive()) {
            float motorPower = gamepad1.left_stick_y;
            motor.setPosition(9);
            telemetry.addData("red", colorSensor.red());
            telemetry.update();

        }

        motor.setPower(0.0);
    }


}
