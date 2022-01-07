package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class New_Super_Class extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        inits();
        waitForStart();
        run();
    }
    public abstract void inits();
    public abstract void run();
}
