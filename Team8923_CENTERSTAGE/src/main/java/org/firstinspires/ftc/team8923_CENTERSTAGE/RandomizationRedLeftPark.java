package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationRedLeftPark")

public class RandomizationRedLeftPark extends BaseAutonomous {
    public void runOpMode() {
        initAuto();
        waitForStart();
        switch (myColorDetection.detectColor()) {
            case ONE:
                driveInches(0, 24);
                pivot(-90);
                // pivot -90 degrees
                runIntake(0.5, 2);
                // output pixel
                pivot(-90);
                //pivot -90 degrees
                driveInches(0, 24);
                // pivot -90 degrees
                driveInches(0, 96);
                break;
            case TWO:
                driveInches(0, 24);
                runIntake(0.5, 2);
                // output pixel
                driveInches(0, -24);
                // pivot 90 degrees
                pivot(90);
                driveInches(0, 96);
                break;
            case THREE:
                driveInches(0, 24);
                // pivot 90 degrees
                pivot(90);
                // output pixel
                runIntake(0.5, 2);
                //pivot 90 degrees
                pivot(90);
                driveInches(0, 24);
                pivot(-90);
                // pivot -90 degrees
                driveInches(0, 96);
                break;
            case FOUR:
                // pivot 90 degrees
                pivot(90);
                driveInches(0, 96);
                break;
        }
    }
}