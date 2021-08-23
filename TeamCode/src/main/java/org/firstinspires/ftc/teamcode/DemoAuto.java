package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;

@Autonomous(name = "Demo Auto")

public class DemoAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        AppContext appContext = AppContext.getInstance();
        appContext.setOpMode(this);
        appContext.init();

        DriveTrain driveTrain = appContext.getDriveTrain();
        driveTrain.driveByTime(1.0, 1000);
    }

}

