package org.firstinspires.ftc.teamcode.mechaninisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotVars;

public class Outtake {

    public final DcMotorEx outtakeMotor;

    private final RobotVars robotVars;

    public Outtake(HardwareMap hardwareMap, RobotVars robotVarsRef) {
        robotVars = robotVarsRef;

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void runOuttake_ToggleMode(boolean outtakeButton, boolean toggleReverseOuttake) {
        if (outtakeButton) {
            robotVars.isOuttakeActive = !robotVars.isOuttakeActive;
        }

        if (robotVars.isOuttakeActive) {
            outtakeMotor.setVelocity(robotVars.constants.outtakeMotorSpeed * (toggleReverseOuttake ? 1 : -0.4));
        } else {
            outtakeMotor.setVelocity(0);
        }
    }



}
