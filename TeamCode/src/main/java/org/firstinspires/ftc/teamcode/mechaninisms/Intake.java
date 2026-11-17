package org.firstinspires.ftc.teamcode.mechaninisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotVars;

public class Intake {

    public final DcMotorEx intakeMotor;

    private final RobotVars robotVars;

    public Intake(HardwareMap hardwareMap, RobotVars robotVarsRef) {
        robotVars = robotVarsRef;

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void runIntake_ToggleMode(boolean intakeButton, boolean toggleReverseIntake) {
        if (intakeButton && !robotVars.isStopperDown) {
            robotVars.isIntakeActive = !robotVars.isIntakeActive;
        }

        if (robotVars.isIntakeActive) {
            intakeMotor.setPower(robotVars.constants.intakeMotorSpeed * (toggleReverseIntake ? -1: 1));
        } else {
            intakeMotor.setPower(0);
        }
    }



}
