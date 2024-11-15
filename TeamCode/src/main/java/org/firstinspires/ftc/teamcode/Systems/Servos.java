package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {

    private Servo[] servos;

    public Servos(HardwareMap hardwareMap) //constructor to init servos
    {
        servos = new Servo[2];

        servos[0] = hardwareMap.get(Servo.class, "L");
        servos[1] = hardwareMap.get(Servo.class, "H");
    }
    public void moveServo(int servoNum, double position) // select a servo and then select a position, you put in 0-360 degrees, and it converts it into 0-1
    {
        double actualPosition = position / 270;

        servos[servoNum].setPosition(actualPosition);
    }

    public double getServoPosition(int servoNum)
    {
        return servos[servoNum].getPosition();
    }

}
