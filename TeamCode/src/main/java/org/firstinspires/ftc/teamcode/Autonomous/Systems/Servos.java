package org.firstinspires.ftc.teamcode.Autonomous.Systems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Servos {

    private Servo lower = null;
    private Servo higher = null;

    public void InitializeServo()
    {
        lower = hardwareMap.get(Servo.class, "L");
        higher = hardwareMap.get(Servo.class, "H");
    }
    public void moveDatServo(double servoDist)
    {
        higher.setPosition(servoDist);
        lower.setPosition(servoDist);
    }
}
