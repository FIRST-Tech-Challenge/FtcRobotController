package org.firstinspires.ftc.teamcode.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {

    private Servo[] servos = null;

    public void InitializeServo()
    {
        servos[0] = hardwareMap.get(Servo.class, "L");
        servos[1] = hardwareMap.get(Servo.class, "H");
    }
    public void moveDatServo(double position, int servoNum)
    {
        double actualPosition = position/360;

        servos[servoNum].setPosition(actualPosition);
    }
}
