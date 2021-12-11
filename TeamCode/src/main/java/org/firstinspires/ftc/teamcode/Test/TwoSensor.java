package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Two sensors", group = "Sensors")

public class TwoSensor extends LinearOpMode {

    DcMotor motor;

    TouchSensor digitalTouchA;
    TouchSensor digitalTouchB;

    @Override
    public void runOpMode() {

        digitalTouchA = hardwareMap.get(TouchSensor.class, "sone_digital");
        digitalTouchB = hardwareMap.get(TouchSensor.class, "stwo_digital");

        waitForStart();

        while (opModeIsActive()) {
            motor = hardwareMap.get(DcMotor.class, "motor");

            if (digitalTouchA.isPressed() && digitalTouchB.isPressed()) {
                telemetry.addData("Both Digital Touches", "Are Pressed");
                motor.setPower(0.3);
            } else {
                telemetry.addData("Both Digital Touches", "Are NOT Pressed");
                motor.setPower(0.0);
            }

            telemetry.update();
        }
    }
}