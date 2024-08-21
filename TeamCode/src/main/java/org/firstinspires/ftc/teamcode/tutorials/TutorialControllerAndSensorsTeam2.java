// Date: 7-23-24

package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Tutorial: Controller and Sensors Turns Motor 2")
public class TutorialControllerAndSensorsTeam2 extends OpMode {
    DcMotor motor;
    TouchSensor samuel;
    ColorSensor bob;
    double power;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motory");
        bob = hardwareMap.get(ColorSensor.class, "colory");
        samuel = hardwareMap.get(TouchSensor.class, "touchy");
    }

    @Override
    public void loop() {
        // If triggers pressed, use those to determine motor speed / direction
        // Right trigger = forward, left trigger = backward
        power = gamepad1.right_trigger - gamepad1.left_trigger;

        // If no triggers pressed, use color sensor. Green = forward. Red = backward.
        if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0)) {
            int bred = bob.red();
            int bgreen = bob.green();
            power = (double)(bgreen - bred) / 10000;
        }

        // If touch sensor pressed, kill switch.
        if(samuel.isPressed()) {
            power = 0;
        }

        telemetry.addData("Red", bob.red());
        telemetry.addData("Green", bob.green());
        telemetry.addData("ButtonPressed", samuel.isPressed());
        telemetry.addData("Power", power);
        telemetry.update();
        motor.setPower(power);
    }
}