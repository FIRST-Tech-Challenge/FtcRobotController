package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Input {

    Motors motors;
    Servos servos;

    public Input(HardwareMap hardwareMap){
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
    }

    public void Move(double power)
    {
        for (int i = 0; i < (motors.motors.length - 2) ; i++) {

            motors.MoveMotor(i, -power); // loop over every motor and move them by gamepad input
        }
    }

    public void Strafe(double power)
    {
        motors.MoveMotor(1, power); // left front
        motors.MoveMotor(3, power); // right front

        motors.MoveMotor(0, -power); // left back
        motors.MoveMotor(2, -power); // right back
    }

    public void Spin(double power)
    {
        motors.MoveMotor(1, power); // left front
        motors.MoveMotor(0, power); // left back

        motors.MoveMotor(2, -power); // right front
        motors.MoveMotor(3, -power); // right back
    }

    public void Intake(double power)
    {
        motors.moveArm(power);
    }

    public void Pickup(boolean button)
    {
        if(button) {
            servos.moveServo(0, 0.25);
        }
        else {
            servos.moveServo(0, 0.7);
        }
    }

    public void armMove(double power)
    {
            motors.MoveMotor(5, power);
    }

    public void drop(boolean dropButton)
    {
        if(!dropButton)
            servos.moveServo(1,0.5);
        else
            servos.moveServo(1,0);
    }
}
