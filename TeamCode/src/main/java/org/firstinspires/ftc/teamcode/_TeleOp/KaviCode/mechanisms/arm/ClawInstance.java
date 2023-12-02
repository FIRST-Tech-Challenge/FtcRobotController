package org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawInstance {
    private HardwareMap hardwareMap;

    public Servo Claw_Top_Finger;

    public Servo Claw_Bottom_Finger;

    private double Claw_Bottom_Open_Position_Value = 0.2;

    private double Claw_Bottom_Close_Position_Value = 0.4;

    private double Claw_Top_Open_Position_Value = 0.3;

    private double Claw_Top_Close_Position_Value = 0.5;

    public void initializeClaw(HardwareMap hardwareMap) {
        Claw_Top_Finger = hardwareMap.get(Servo.class, "Claw_Top_Finger");
        Claw_Bottom_Finger = hardwareMap.get(Servo.class, "Claw_Bottom_Finger");

        Claw_Top_Finger.setDirection(Servo.Direction.FORWARD);
        Claw_Bottom_Finger.setDirection(Servo.Direction.FORWARD);

        Claw_Top_Finger.scaleRange(0, 1);
        Claw_Bottom_Finger.scaleRange(0, 1);
    }

    public void Actuate_Claw_Top_Finger(String Action) {
        if (Action == "open") {
            Claw_Top_Finger.setPosition(Claw_Top_Open_Position_Value);
        } else if (Action == "close") {
            Claw_Top_Finger.setPosition(Claw_Top_Close_Position_Value);
        }
    }

    public void Actuate_Claw_Bottom_Finger(String Action) {
        if (Action == "close") {
            Claw_Bottom_Finger.setPosition(Claw_Bottom_Open_Position_Value);
        } else if (Action == "open") {
            Claw_Bottom_Finger.setPosition(Claw_Bottom_Close_Position_Value);
        }
    }
}
