package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class StackBreaker {

    private Servo servo;

    public static double CLOSE_POSITION = 0.3;
    public static double OPEN_POSITION = 0;


    public StackBreaker(LinearOpMode pup) {
        servo = pup.hardwareMap.get(Servo.class,"stBreak");
        servo.setPosition(CLOSE_POSITION);
    }

    public void switchPosition() {
        if (servo.getPosition() == CLOSE_POSITION) {
            servo.setPosition(OPEN_POSITION);
        } else {
            servo.setPosition(CLOSE_POSITION);
        }
    }
}
