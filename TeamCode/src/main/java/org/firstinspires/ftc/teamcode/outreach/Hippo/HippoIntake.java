package org.firstinspires.ftc.teamcode.outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.controller.MotorControl;

public class HippoIntake extends MotorControl
{
    // + speed is intaking
    public HippoIntake(OpMode opMode)
    {
        super("intake", true, false, opMode);
    }

    @Override
    protected void motorBreak()
    {
        super.motorBreak();
    }

    @Override
    protected void simpleDrive(double speed, boolean argument1, boolean argument2)
    {
        super.simpleDrive(speed, argument1, argument2);
    }
}
