package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Input {

    Motors motors;
    Servos servos;

    public Input(HardwareMap hardwareMap) {
        motors = new Motors(hardwareMap);
        servos = new Servos(hardwareMap);
    }

    public void move(double power) {

        motors.MoveMotor(Motors.Type.LeftBack, -power);
        motors.MoveMotor(Motors.Type.LeftFront, -power);
        motors.MoveMotor(Motors.Type.RightFront, -power);
        motors.MoveMotor(Motors.Type.RightBack, -power);

    }

    public void strafe(double power) {
        motors.MoveMotor(Motors.Type.LeftFront, power); // left front
        motors.MoveMotor(Motors.Type.RightFront, power); // right front

        motors.MoveMotor(Motors.Type.LeftBack, -power); // left back
        motors.MoveMotor(Motors.Type.LeftBack, -power); // right back
    }

    public void spin(double power) {
        motors.MoveMotor(Motors.Type.RightFront, power); // left front
        motors.MoveMotor(Motors.Type.LeftBack, power); // left back

        motors.MoveMotor(Motors.Type.RightFront, -power); // right front
        motors.MoveMotor(Motors.Type.RightBack, -power); // right back
    }

    public void claw(boolean grabButton, boolean releaseButton) {

        if (releaseButton) {
            servos.moveServo(Servos.Type.Claw, 1);
        }
        else if (grabButton) {
            servos.moveServo(Servos.Type.Claw, 0.645);
        }

    }

    public void upArm(double power) {
        motors.MoveMotor(Motors.Type.Pull, power);
    }
}
