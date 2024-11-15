package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Input {

    Motors motors;

    public Input(HardwareMap hardwareMap)
    {
        motors = new Motors(hardwareMap);
    }

    public void Move(double power)
    {

        for (int i = 0; i < motors.driveTrainMotors.length; i++) {

            motors.MoveMotor(i, power); // loop over every motor and move them by gamepad input
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

    public void Arm(double power)
    {
        motors.MoveMotor(4,power);
    }
}
