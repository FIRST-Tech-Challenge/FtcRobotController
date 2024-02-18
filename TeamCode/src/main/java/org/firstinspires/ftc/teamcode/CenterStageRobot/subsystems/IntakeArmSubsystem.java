package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeArmSubsystem extends SubsystemBase {
    private double MIN = 0.1,MID = 0.1, MAX = 0.65;
    private final ServoImplEx rightArm;

    public IntakeArmSubsystem(HardwareMap hm) {
        rightArm = hm.get(ServoImplEx.class, "right_intake_arm");

        setPosition(MAX);
    }

    public void setPosition(double position) {
        rightArm.setPosition(1-position);
    }

    public void raiseArm() {
        setPosition(MAX);
    }

    public void lowerArm() {
        setPosition(MIN);
    }

    public void midArm() {
        setPosition(MID);
    }
}
