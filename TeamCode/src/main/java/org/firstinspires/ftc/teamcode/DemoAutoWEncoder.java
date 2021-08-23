package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drivetrain.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Constants;


@Autonomous(name = "Demo Auto w Encoder")

public class DemoAutoWEncoder extends LinearOpMode {
    @Override
    public void runOpMode() {
        AppContext appContext = AppContext.getInstance();
        appContext.setOpMode(this);
        appContext.init();

        DriveTrain driveTrain = appContext.getDriveTrain();

        // Go in a square
        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 10, -10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 10, -10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, 10, -10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -10, 10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -10, 10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -10, 10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);
        driveTrain.drive(Constants.TURN_SPEED, -10, 10, 4.0);

        driveTrain.drive(Constants.DRIVE_SPEED, 15, 15, 5.0);


        // Spin in a circle
        //driveTrain.drive(DRIVE_SPEED,  -33.5,  33.5, 5.0);

    }

}
