package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DoubleHorizontalExtendo {
    private DcMotor leftMotor;
    private DcMotor rightMotor;

    public DoubleHorizontalExtendo(HardwareMap hw, String leftName, String rightName)
    {
        leftMotor = hw.get(DcMotor.class, leftName);
        rightMotor = hw.get(DcMotor.class, leftName);
    }

    public void setPower(double power)
    {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }

    public double getLeftPower()
    {
        return leftMotor.getPower();
    }

    public double getRightPower()
    {
        return rightMotor.getPower();
    }
}
