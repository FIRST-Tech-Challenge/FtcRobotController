package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
class WobbleArmServoVals {
    public static double position = 0;
}

@TeleOp(name = "WobbleArmServoTest")
public class WobbleArmTest extends OpMode {

    double motorPosition = 0.0;
    private Motor wobbleArmMotor;

    double servoPosition = 0.0;
    private Servo wobbleArmServo;

    @Override
    public void init() {

        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        wobbleArmMotor = new Motor (hardwareMap, "wobbleArmMotor");
    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            servoPosition = 1.0;

        } else {

            servoPosition = 0.0;
        }

        wobbleArmServo.setPosition(servoPosition);

        if (gamepad1.dpad_up) {

            motorPosition = 0.625;

        } else if (gamepad1.dpad_down) {

            motorPosition= 0.0;

        }

        wobbleArmMotor.setTargetDistance(motorPosition);

    }
}



