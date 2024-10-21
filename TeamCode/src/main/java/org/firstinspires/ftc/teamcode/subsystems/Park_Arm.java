package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Park_Arm {
    private Servo Park_Arm;

    public Park_Arm(HardwareMap hardwareMap) {
        this.Park_Arm = hardwareMap.get(Servo.class, "Park_Arm");
        this.Park_Arm.setDirection(Servo.Direction.FORWARD);
    }

    public void setPosition(double position) {
        Park_Arm.setPosition(position);
    }
}
