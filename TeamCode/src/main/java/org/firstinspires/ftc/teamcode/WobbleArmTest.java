package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot_utilities.Vals;

@Config
class WobbleArmServoVals {
    public static double position = 0;
}

@TeleOp(name = "WobbleArmServoTest")
public class WobbleArmTest extends OpMode {

    double motorPosition = Vals.initialMotorPosition;
    double servoPosition = Vals.initialServoPosition;
    private Motor wobbleArmMotor;
    private Servo wobbleArmServo;

    @Override
    public void init() {

        wobbleArmServo = hardwareMap.servo.get("wobbleArmServo");
        wobbleArmMotor = new Motor(hardwareMap, "wobbleArmMotor");
    }

    @Override
    public void loop() {

        // Servo motor.
        if (gamepad1.a) {

            servoPosition = Vals.servoOpenUp;

        } else {

            servoPosition = Vals.servoCloseUp;
        }

        wobbleArmServo.setPosition(servoPosition);

        // Motor movement.
        if (gamepad1.dpad_up) {

            motorPosition = Vals.motorMoveUp;

        } else {

            motorPosition = Vals.motorReturnToDefault;

        }

        wobbleArmMotor.set(motorPosition);

        if (gamepad1.dpad_down) {

            motorPosition = Vals.motorMoveDown;

        } else {

            motorPosition = Vals.motorReturnToDefault;

        }

        wobbleArmMotor.set(motorPosition);

    }
}



