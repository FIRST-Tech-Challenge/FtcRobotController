package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
/**
 * Testing for any generic DC Motor.
 * Make sure the motor is named TEST, all caps.
 *
 * Would be much smarter if you swapped out the wires instead
 * */
@TeleOp(name = "GenericMotorTesting", group = "Test Programs")
public class GenericMotorTesting extends OpMode {
    public DcMotorEx testMotor;
    @Override
    public void init() {
        testMotor = (DcMotorEx) hardwareMap.get(DcMotor.class, "TEST");
    }

    @Override
    public void loop() {
        double yInput = gamepad1.left_stick_y;
        telemetry.addData("Input", yInput);
        testMotor.setPower(yInput);
        telemetry.update();
    }
}
