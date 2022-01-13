package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Test_MyFirstOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor motor = hardwareMap.get(DcMotor.class,"motor");
        ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        waitForStart();
        while (opModeIsActive()) {
            float motorPower = gamepad1.left_stick_y;
            motor.setPower(motorPower);
            telemetry.addData("red", colorSensor.red());
            telemetry.update();

        }

        motor.setPower(0.0);
    }


}
