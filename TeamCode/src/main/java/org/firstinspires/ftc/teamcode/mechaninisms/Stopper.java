package org.firstinspires.ftc.teamcode.mechaninisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotVars;

public class Stopper {

    public final Servo stopperServo;

    private final RobotVars robotVars;
    private final Outtake outtake;

    public Stopper(HardwareMap hardwareMap, RobotVars robotVarsRef, Outtake outtakeRef) {
        robotVars = robotVarsRef;
        outtake = outtakeRef;

        stopperServo = hardwareMap.get(Servo.class, "stopper");
    }


    public void runStopper_ToggleMode(boolean shootButton) {
        if (shootButton) {
            robotVars.isStopperDown = true;

            stopperServo.setPosition(robotVars.constants.stopperDownPosition);

        } else {
            robotVars.isStopperDown = false;
            stopperServo.setPosition(robotVars.constants.stopperUpPosition);
        }
    }

}
