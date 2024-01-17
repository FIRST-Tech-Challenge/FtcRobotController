package org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Winch extends SubsystemBase {
    private DcMotorEx winchMotor;

    public Winch(HardwareMap hardwareMap) {
        winchMotor = hardwareMap.get(DcMotorEx.class, "winch");

        winchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        winchMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        // Add any periodic tasks or checks here
    }

    public void extendWinch(double power) {
        winchMotor.setPower(power);
    }

    public void retractWinch(double power) {
        winchMotor.setPower(-power);
    }

    public void stopWinch() {
        winchMotor.setPower(0);
    }
}
