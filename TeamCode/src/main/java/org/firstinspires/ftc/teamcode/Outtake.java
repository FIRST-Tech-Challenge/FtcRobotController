package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Outtake {
    DcMotor outtakeMotor;
    Servo droneServo, trapdoorServo;
    ElapsedTime trapdoorTime;
    boolean trapdoor = false;
    boolean drone = false;

    public Outtake(HardwareMap hwMap) {
        outtakeMotor = hwMap.get(DcMotor.class, "outtakeMotor");

        outtakeMotor.setDirection(DcMotor.Direction.REVERSE);

        droneServo = hwMap.get(Servo.class, "droneServo");
        trapdoorServo = hwMap.get(Servo.class, "trapdoorServo");
    }

    public void driveLift(double power) {
        outtakeMotor.setPower(power);
    }

    public void launchDrone(boolean open) {
        if (open) {
            if (drone) {
                drone = false;
                droneServo.setPosition(1.0);
            }
            else {
                drone = true;
                droneServo.setPosition(0.0);
            }
        }


    }

    public void trapdoor(boolean open) {
        if (open) {
            if (trapdoor) {
                trapdoor = false;
                trapdoorServo.setPosition(1.0);
            }
            else {
                trapdoor = true;
                trapdoorServo.setPosition(0.0);
            }
        }


    }
}
