package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeArmSubsystem extends SubsystemBase {
    private double MIN = 0.1,MID = 0.1, MAX = 0.65;

    private double PIXEL_LOCK_POS = 0.05;
    private double[] AUTO = {0.1, 0.16, 0.22, 0.255, 0.30, 0.55};
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

    public void midArm() {
        setPosition(MID);
    }

    public void lowerArm() {
        setPosition(MIN);
    }

    public void auto_pixel(int index){
        setPosition(AUTO[index - 1]);
    }

    public void lockPixel() {
        setPosition(PIXEL_LOCK_POS);
    }
}
