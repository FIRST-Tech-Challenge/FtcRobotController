package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationBlueLeftPark")

public class RandomizationBlueLeftPark extends BaseAutonomous {
    public void runOpMode() {
        initAuto();
        waitForStart();
        switch (myColorDetection.detectColor()) {
            case ONE:
                driveInches(0, 24);
                // pivot -90 degrees
                // output purple pixel
                // pivot 90 degrees
                driveInches(0, -24);
                // pivot -90 degrees
                driveInches(0, 48);
                break;
            case TWO:
                driveInches(0, 24);
                // output purple pixel
                driveInches(0, -24);
                // pivot -90 degrees
                driveInches(0, 48);
                break;
            case THREE:
                driveInches(0, 24);
                // pivot 90 degrees
                // output purple pixel
                // pivot -90 degrees
                driveInches(0, -24);
                // pivot -90 degress
                driveInches(0, 48);
                break;
            case FOUR:
                driveInches(0, 4);
                //pivot -90 degrees
                driveInches(0, 24);
                break;
        }
    }
}
