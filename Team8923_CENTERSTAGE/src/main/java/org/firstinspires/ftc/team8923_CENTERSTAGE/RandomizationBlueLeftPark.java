package org.firstinspires.ftc.team8923_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
@Autonomous (name = "RandomizationBlueLeftPark")

public class RandomizationBlueLeftPark extends BaseAutonomous {
    public void runOpMode() {
        initAuto();
        waitForStart();
        /*switch (myColorDetection.detectColor()) {
            case ONE:
                // forward 1 tile
                driveInches(0, 24);
                // pivot 90 degrees left
                pivot(-90);
                // release pixel
                runIntake(0.5, 2);
                // pivot 90 degrees right
                pivot(90);
                // backward 1 tile
                driveInches(0, -24);
                // pivot 90 degrees left
                pivot(-90);
                // forward 2 tiles
                driveInches(0, 48);
                break;
            case TWO:
                // forward 1 tile
                driveInches(0, 24);
                // release pixel
                runIntake(0.5, 2);
                // backward 1 tile
                driveInches(0, -24);
                // pivot 90 degrees left
                pivot(-90);
                // forward 2 tiles
                driveInches(0, 48);
                break;
            case THREE:
                // forward 1 tile
                driveInches(0, 24);
                // pivot 90 degrees right
                pivot(90);
                // release pixel
                runIntake(0.5, 2);
                // pivot 90 degrees left
                pivot(-90);
                // backward 1 tile
                driveInches(0, -24);
                // pivot 90 degrees left
                pivot(-90);
                // forward 2 tiles
                driveInches(0, 48);
                break;
            case FOUR:
                // this is just parking tbh ngl (forward 4 inches, pivot left, forward 1 tiles)
                driveInches(0, 4);
                pivot(-90);
                driveInches(0, 24);
                break;
        }*/
    }
}
