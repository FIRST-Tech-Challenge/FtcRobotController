// Date: 7-23-24

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Tutorial: Controller And Sensors Turn Motor")
public class ControllerAndSensorsSolution extends OpMode {
    DcMotor motor;
    ColorSensor colorSensor;
    TouchSensor touchSensor;
//    double power;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motory");
        colorSensor = hardwareMap.get(ColorSensor.class, "colory");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchy");
    }

    @Override
    public void loop() {
        double power = 0;
        // 1. Get the power from the controller
        // 2. Show the power on the Android device in real time
        // 3. Turn the motor by that power value

        // Option 1: Do everything in the loop method
        // Great for simple processes
        if (touchSensor.getValue() ==1) {
            return;
        }
        power = gamepad1.right_trigger - gamepad1.left_trigger;
        telemetry.addData("Gamepad Power", power);
        telemetry.update();

        if (power == 0) {
            int red = colorSensor.red();
            int green = colorSensor.green();
            double difference = green - red;
            power = difference / 255;
        }

        motor.setPower(power);

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