// Date: 7-23-24

package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Tutorial: Controller and Sensors Turns Motor")
public class TutorialControllerAndSensorsTeam1 extends OpMode {
    DcMotor motor;
    TouchSensor touchSensor;
    ColorSensor jimmy;
    double power;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motory");
        jimmy = hardwareMap.get(ColorSensor.class, "colory");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchy");
    }

    @Override
    public void loop() {
        // If triggers pressed, use those to determine motor speed / direction
        // Right trigger = forward, left trigger = backward
        power = gamepad1.right_trigger - gamepad1.left_trigger;

        // If no triggers pressed, use color sensor. Green = forward. Red = backward.
        if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0)) {
            int bob = jimmy.red();
            int tim = jimmy.green();
            power = (double)(tim - bob) / 255;
        }

        // If touch sensor pressed, kill switch.
        if(touchSensor.isPressed()) {
            power = 0;
        }

        telemetry.addData("Red", jimmy.red());
        telemetry.addData("Green", jimmy.green());
        telemetry.addData("ButtonPressed", touchSensor.isPressed());
        telemetry.addData("Power", power);
        telemetry.update();
        motor.setPower(power);
    }
}