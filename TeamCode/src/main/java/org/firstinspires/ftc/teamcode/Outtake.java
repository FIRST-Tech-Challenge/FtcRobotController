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
    boolean trapToggle = false;
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


    public void trapdoor(boolean button, ElapsedTime time) {
        if (button && time.time() > .25 && !trapToggle) {
            trapToggle = true;
            time.reset();
            trapdoorServo.setPosition(0.0);

        }
        else if (button && time.time() > .25 && trapToggle) {
            trapToggle = false;
            time.reset();
            trapdoorServo.setPosition(1.0);

        }


    }
}
