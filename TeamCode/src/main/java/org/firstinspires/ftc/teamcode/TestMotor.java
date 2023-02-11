package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Motor Test", group="Tests")
@Disabled
public class TestMotor extends LinearOpMode {

    DcMotorEx motor1;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotorEx motor4;
    SimpleServo servo1;
    SimpleServo servo2;
    GamepadEx gp;

    double servo1Pos = 0;
    double servo2Pos = 0;
    @Override
    public void runOpMode() {
        gp = new GamepadEx(gamepad1);

        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor4 = hardwareMap.get(DcMotorEx.class, "motor4");

        servo1 = new SimpleServo(hardwareMap, "servo1", 0.0,
                180.0, AngleUnit.DEGREES);
        servo2 = new SimpleServo(hardwareMap, "servo2", 0.0,
                180.0, AngleUnit.DEGREES);

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.dpad_right) servo1Pos += 0.002;
            if(gamepad1.dpad_left) servo1Pos -= 0.002;
            if(gamepad1.dpad_up) servo2Pos += 0.002;
            if(gamepad1.dpad_down) servo2Pos -= 0.002;

            motor1.setPower(gp.getLeftY());
            motor2.setPower(gp.getRightY());
            motor3.setPower(gp.getLeftX());
            motor4.setPower(gp.getRightX());

            servo1.setPosition(servo1Pos);
            servo2.setPosition(servo2Pos);


            telemetry.addData("Power Motor 1: ", gp.getLeftY());
            telemetry.addData("Power Motor 2: ", gp.getRightY());
            telemetry.addData("Power Motor 3: ", gp.getLeftX());
            telemetry.addData("Power Motor 4: ", gp.getRightX());
            telemetry.addData("Servo 1 Position: ", servo1Pos);
            telemetry.addData("Servo 2 Position: ", servo2Pos);
            telemetry.update();
        }
    }
}
