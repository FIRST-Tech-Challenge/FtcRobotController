package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teamUtil.configNames;
import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

public class intake {
    robotConfig r;
    Servo intake;
    private double intakePos;

    public intake(robotConfig r) {
        this.r = r;

        intake = r.hardwareMap.get(Servo.class, configNames.intake);
        intake.scaleRange(robotConstants.intakeOpen, robotConstants.intakeClosed);
        setPos(0);
    }

    /**
     sets the wrist target position for the wrist servo.
     robotConstants.wristFront and robotConstants.wristBack have been set to 0 and 1.0 respectively
     */
    public void setPos(double targetPos)    {
        intakePos = targetPos;
    }

    public void update(){
        intake.setPosition(intakePos);
    }

}
