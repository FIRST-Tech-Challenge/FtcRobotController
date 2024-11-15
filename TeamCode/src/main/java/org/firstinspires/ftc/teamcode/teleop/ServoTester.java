package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTester {
    public void runOpMode() throws InterruptedException {

        final Servo rightWristServo;
        rightWristServo = hardwareMap.get(Servo.class, "rightWristServo");
        int SERVO_POSITION= 0;
                rightWristServo.setPosition(SERVO_POSITION);

    }

}
