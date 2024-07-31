// Date: 7-23-24

package org.firstinspires.ftc.teamcode.tutorials;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Tutorial: Controller Turns Motor")
public class TutorialControllerTurnMotor extends OpMode {
    DcMotor motor;
    double power;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "motory");
        power = 0;
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