package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Input {

    Motors motors;

    public Input(HardwareMap hardwareMap){
        motors = new Motors(hardwareMap);
    }

    public void Move(double power)
    {
        for (int i = 0; i < (motors.motors.length - 1) ; i++) {

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
        motors.MoveMotor(4,power / 2);

        if(motors.GetArmDistance() <= 0 && power <= 0)
        {
            motors.setArmPosition(motors.getArmPosition());
        }

    }
}
