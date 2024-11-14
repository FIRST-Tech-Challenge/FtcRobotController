package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.arcrobotics.ftclib.controller.PIDFController;

public class Drive extends LinearOpMode {
    private ElapsedTime timer;
    private SwerveDrivetrain drivetrain;
    private PIDFController pidfControler = new PIDFController(0.5, 0, 0.5, 0);

    @Override
    public void runOpMode() {
        drivetrain = new SwerveDrivetrain();

    }

}
