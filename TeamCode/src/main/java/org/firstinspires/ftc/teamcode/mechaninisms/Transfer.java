package org.firstinspires.ftc.teamcode.mechaninisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotVars;

public class Transfer {

    public final DcMotorEx transferMotor;

    private final RobotVars robotVars;

    public Transfer(HardwareMap hardwareMap, RobotVars robotVarsRef) {
        robotVars = robotVarsRef;

        transferMotor = hardwareMap.get(DcMotorEx.class, "transfer");
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void runTransfer_ToggleMode(boolean transferButton, boolean toggleReverseIntake) {
        if (robotVars.isIntakeActive) {
            transferMotor.setPower(robotVars.constants.transferMotorSpeed * (toggleReverseIntake ? -1 : 1));
        } else {
            transferMotor.setPower(0);
        }
    }



}
