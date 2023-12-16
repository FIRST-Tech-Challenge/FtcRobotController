package org.firstinspires.ftc.teamcode._TeleOp.KaviCode.mechanisms.arm;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawInstance {
    private HardwareMap hardwareMap;

    public Servo Claw_Top_Finger;

    public Servo Claw_Bottom_Finger;

    private double Claw_Bottom_Open_Position_Value = 0.4;

    private double Claw_Bottom_Close_Position_Value = 0;

    private double Claw_Top_Open_Position_Value = 0.4;

    private double Claw_Top_Close_Position_Value = 0;

    public String Claw_Top_Status = "";
    public String Claw_Bottom_Status = "";

    public void initializeClaw(HardwareMap hardwareMap) {
        Claw_Top_Finger = hardwareMap.get(Servo.class, "Claw_Top_Finger");
        Claw_Bottom_Finger = hardwareMap.get(Servo.class, "Claw_Bottom_Finger");

        Claw_Top_Finger.setDirection(Servo.Direction.REVERSE);
        Claw_Bottom_Finger.setDirection(Servo.Direction.FORWARD);

        Claw_Top_Finger.scaleRange(0, 1);
        Claw_Bottom_Finger.scaleRange(0, 1);

        Actuate_Claw_Top_Finger("close");
        Actuate_Claw_Bottom_Finger("close");
    }

    public void Actuate_Claw_Top_Finger(String Action) {
        if (Action == "open") {
            Claw_Top_Finger.setPosition(Claw_Top_Open_Position_Value);
            Claw_Top_Status = "open";
        } else if (Action == "close") {
            Claw_Top_Finger.setPosition(Claw_Top_Close_Position_Value);
            Claw_Top_Status = "closed";
        } else if (Action == "toggle") {
            if (Claw_Top_Status == "open") {
                Actuate_Claw_Top_Finger("close");
            } else if (Claw_Top_Status == "closed") {
                Actuate_Claw_Top_Finger("open");
            }
        }
    }

    public void Actuate_Claw_Bottom_Finger(String Action) {
        if (Action == "open") {
            Claw_Bottom_Finger.setPosition(Claw_Bottom_Open_Position_Value);
            Claw_Bottom_Status = "open";
        } else if (Action == "close") {
            Claw_Bottom_Finger.setPosition(Claw_Bottom_Close_Position_Value);
            Claw_Bottom_Status = "closed";
        } else if (Action == "toggle") {
            if (Claw_Bottom_Status == "open") {
                Actuate_Claw_Bottom_Finger("close");
            } else if (Claw_Bottom_Status == "closed") {
                Actuate_Claw_Bottom_Finger("open");
            }
        }
    }
}