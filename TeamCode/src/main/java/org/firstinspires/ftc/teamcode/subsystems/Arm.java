package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm extends SubsystemBase {
    private Servo armServo;

    public enum ArmState {
        INTAKE,
        SCORE
    }

    public Arm(HardwareMap hMap) {
        this.armServo = hMap.get(Servo.class, "Arm");
    }


    public void goToPos(ArmState state) {
        switch(state) {
            case SCORE:
                this.armServo.setPosition(1);
                break;
            case INTAKE:
                this.armServo.setPosition(0);
                break;
        }
    }



}
