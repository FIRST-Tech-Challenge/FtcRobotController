package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class TempArmBot extends DuckBot {
    public Servo tempArm = null;

    //two positions of the wobble servo
    final double armPinched = 0.655;
    final double armOpened = 0.7;

    public boolean isOpen = true;
    long lastToggleDone = 0;
    long timeSinceToggle = 0;
    long lastPosSwitch = 0;
    long timeSincePosSwitch = 0;

    public TempArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        tempArm = hwMap.get(Servo.class, "tempArm");
        tempArm.setPosition(armPinched);
    }

    //call openPinch() to open the arm
    public void openPinch() {
        tempArm.setPosition(armOpened);
    }

    //call closeArm() to close the arm
    public void closePinch() {
        tempArm.setPosition(armPinched);
    }

    public void toggleWobble(boolean button) {
        timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 300) {
            if (isOpen) {
                tempArm.setPosition(armPinched);
                isOpen = false;
                lastToggleDone = System.currentTimeMillis();
            } else if (!isOpen) {
                tempArm.setPosition(armOpened);
                isOpen = true;
                lastToggleDone = System.currentTimeMillis();
            }
        }
    }
}
