package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
class WobbleArmServoVals {
    public static double position = 0;
}

@TeleOp(name = "WobbleArmServoTest")
public class WobbleArmTest extends OpMode {

    double servoPosition = 0.0;
    private Servo wobbleArmServo;

    @Override
    public void init() {
        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");

    }

    @Override
    public void loop() {

        if (gamepad1.a) {

            servoPosition = 1.0;

        } else {

            servoPosition = 0.0;
        }

        wobbleArmServo.setPosition(servoPosition);
    }
}



