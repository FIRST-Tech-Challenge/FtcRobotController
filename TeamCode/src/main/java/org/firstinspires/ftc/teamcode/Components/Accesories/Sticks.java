package org.firstinspires.ftc.teamcode.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot;

public class Sticks {
    final Servo leftStick;
    final Servo rightStick;

    public Sticks(LinearOpMode opMode) {
        LinearOpMode op = opMode;
        leftStick = op.hardwareMap.servo.get("leftStick");
        rightStick = op.hardwareMap.servo.get("rightStick");

        moveLeftStick(1);
        moveRightStick(0);
        if(Robot.isCorgi){
            moveLeftStick(0);
            moveRightStick(1);
        }

    }

    public void moveLeftStick(double distance) {
        leftStick.setPosition(distance);
    }

    public void moveRightStick(double distance) {
        rightStick.setPosition(distance);
    }
}
