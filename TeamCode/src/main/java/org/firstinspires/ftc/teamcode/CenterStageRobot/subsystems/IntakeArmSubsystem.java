package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class IntakeArmSubsystem extends SubsystemBase {
//    private double MIN = 0.02,MID = 0.1, MAX = 5;
    private double[] AUTO = {0.12, 0.17, 0.22, 0.28, 0.32, 0.55};
//    private final ServoEx leftArm;
    private final ServoImplEx rightArm;

    public IntakeArmSubsystem(HardwareMap hm) {
//        leftArm = new SimpleServo(hm, "left_intake_arm", 0, 180, AngleUnit.DEGREES); // TODO: Angle Values Might Change
        rightArm = hm.get(ServoImplEx.class, "right_intake_arm");
        raiseArm();
    }

    public void setPosition(double position) {
//        leftArm.setPosition(position);
        rightArm.setPosition(1-position);
    }

    public void auto_pixel(int index){
        setPosition(AUTO[index - 1]);
    }

    public void raiseArm() {
        setPosition(AUTO[5]);; // TODO: Might be 0
    }

    public void lowerArm() {
        setPosition(AUTO[0]); // TODO: Might be 1
    }
}
