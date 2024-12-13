package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Date;

public class Input {

    Motors motors;
    Servos servos;

    public static double MAX_SPEED = 20.0;
    public static int POS_0 = -460;
    public static int POS_1 = -65;
    public static int POS_3 = 580;

    ElapsedTime elapsedTime = new ElapsedTime();

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
            servos.moveServo(0, 1);
        }
        else if (grabButton) {
            servos.moveServo(0, 0.645);
        }

    }

    public void upArm(double power) {
        motors.MoveMotor(Motors.Type.Claw, power);
    }
}
