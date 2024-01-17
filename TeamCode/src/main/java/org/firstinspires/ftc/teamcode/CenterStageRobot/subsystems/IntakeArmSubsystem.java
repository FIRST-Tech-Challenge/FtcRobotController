package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeArmSubsystem extends SubsystemBase {
    private double MIN = 0.02,MID = 0.1, MAX = 0.55;
//    private final ServoEx leftArm;
    private final ServoImplEx rightArm;

    public IntakeArmSubsystem(HardwareMap hm) {
//        leftArm = new SimpleServo(hm, "left_intake_arm", 0, 180, AngleUnit.DEGREES); // TODO: Angle Values Might Change
        rightArm = hm.get(ServoImplEx.class, "right_intake_arm");

        setPosition(MAX);
    }

    public void setPosition(double position) {
//        leftArm.setPosition(position);
        rightArm.setPosition(1-position);
    }

    public void raiseArm() {
        setPosition(MAX); // TODO: Might be 0
    }

    public void lowerArm() {
        setPosition(MIN); // TODO: Might be 1
    }

    public void midArm() {
        setPosition(MID); // TODO: Might be 1
    }
}
