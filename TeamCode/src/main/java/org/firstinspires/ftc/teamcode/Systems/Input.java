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

    public void intake(double power) {

        motors.MoveMotor(Motors.Type.Arm, power);
    }

    public void pickup(boolean button) {
        if(button) {
            servos.moveServo(0, 0.25);
        }
        else {
            servos.moveServo(0, 0.7);
        }
    }

    public void armMove(double power) {
        double speed;

        if(power > MAX_SPEED) {
            speed = MAX_SPEED;
        } else {
            speed = power;
        }

        motors.MoveMotor(Motors.Type.Arm, speed);
    }

    public void drop(boolean dropButton) {

        if(!dropButton)
            servos.moveServo(1,0.5);
        else
            servos.moveServo(1,0);
    }

    public void hang(boolean hangButton) {
        if (hangButton) {
            motors.MoveMotor(Motors.Type.Claw, 50);
        }
    }

    public void stabalizeArm(double power) {

        double armPos = motors.getArmPosition();


//        if (armPos > -460 && armPos < -381) {
//
//        } else if (armPos > -460 && armPos < -381) {
//
//        } else if (armPos > -460 && armPos < -381) {
//
//        } else if (armPos > -460 && armPos < -381) {
//
//        }
//
//
//
//
//        if (armPos < -65 && armPos > )


        elapsedTime.reset();
        /* back is about -460
           neutral is -65
           and front is 580

         */


        double time1 = elapsedTime.milliseconds();
        double armPos1 = motors.getArmPosition();

        double time2 ;
        double armPos2;

        //motors.MoveMotor(Motors.Type.Arm, power);

        while (Math.abs(power) > 0.1) {

            time2 = elapsedTime.milliseconds();
            armPos2 = motors.getArmPosition();

            double velocity = (armPos2 - armPos1) / (time2 - time1);

            if (velocity > 20) {
                power++;
            } else {
                power--;
            }

            time1 = time2;
            armPos1 = armPos2;

            motors.MoveMotor(Motors.Type.Arm, power);
        }

        //find slope / arm velocity then set power so it always is moving at the same speed


    }
}
