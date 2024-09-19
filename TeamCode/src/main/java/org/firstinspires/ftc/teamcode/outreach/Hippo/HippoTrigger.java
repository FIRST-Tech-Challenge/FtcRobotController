package org.firstinspires.ftc.teamcode.outreach.Hippo;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.controller.ServoControl;

public class HippoTrigger extends ServoControl
{
    // the smalled the value the more the trigger will be extended, the larger the more it will be contracted
    public HippoTrigger(OpMode opMode)
    {
        super("trigger", 0, 1, opMode);
    }

    @Override
    protected void driveServo(double target)
    {
        super.driveServo(target);
    }
}
