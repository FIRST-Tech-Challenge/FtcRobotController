// Date: 7-23-24

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Tutorial: Controller Turns Motor")
public class TutorialControllerAndSensorsTurnMotor extends OpMode {
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
        // 1. Get the power from the controller
        // 2. Show the power on the Android device in real time
        // 3. Turn the motor by that power value

        // Option 1: Do everything in the loop method
        // Great for simple processes
        power = gamepad1.right_trigger - gamepad1.left_trigger;
        telemetry.addData("Power", power);
        telemetry.update();
        motor.setPower(power);

        // If triggers pressed, use those to determine motor speed / direction
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


        // Option 2: Using individual methods
        // Great for organization and separating complexity
        // power = getPower();
        // showPower();
        // turnMotor();
    }

    private double getPower() {
        return gamepad1.right_trigger - gamepad1.left_trigger;
    }

    private void showPower() {
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private void turnMotor() {
        motor.setPower(power);
    }
}