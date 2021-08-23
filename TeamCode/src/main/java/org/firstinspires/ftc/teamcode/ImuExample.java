package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;

@Autonomous(name = "ImuExample")

public class ImuExample extends LinearOpMode {

    @Override
    public void runOpMode() {
        AppContext appContext = AppContext.getInstance();
        appContext.setOpMode(this);
        appContext.init();

        DriveTrain driveTrain = appContext.getDriveTrain();

        driveTrain.driveStraight(1, 500);
        driveTrain.turnLeft(92);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turnRight(0);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turnRight(-25);
        driveTrain.driveStraight(1, 1200);

        driveTrain.turnLeft(0);
        driveTrain.driveStraight(-1, 1864);
        driveTrain.turnRight(-90);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turnLeft(0);
        driveTrain.driveStraight(1, 1100);
        driveTrain.turnLeft(25);
        driveTrain.driveStraight(1, 1200);

    }


}


