package org.firstinspires.ftc.teamcode.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.VuforiaNav;

@TeleOp(name = "MezCamera", group = "Concept")
public class OpModeMez extends OpMode
{

    protected VuforiaNav mVufoiraNav;

    @Override
    public void init()
    {
        mVufoiraNav = new VuforiaNav(this.hardwareMap);
    }



    @Override
    public void loop()
    {

    }
}

