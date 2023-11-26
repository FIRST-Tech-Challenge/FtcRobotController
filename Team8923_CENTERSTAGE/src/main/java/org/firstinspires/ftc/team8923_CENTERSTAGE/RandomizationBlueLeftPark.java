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
                pivot(-90);
                runIntake(0.5, 2);
                pivot(90);
                driveInches(0, -24);
                pivot(-90);
                driveInches(0, 48);
                break;
            case TWO:
                driveInches(0, 24);
                runIntake(0.5, 2);
                driveInches(0, -24);
                pivot(-90);
                driveInches(0, 48);
                break;
            case THREE:
                driveInches(0, 24);
                pivot(90);
                runIntake(0.5, 2);
                pivot(-90);
                driveInches(0, -24);
                pivot(-90);
                driveInches(0, 48);
                break;
            case FOUR:
                driveInches(0, 4);
                pivot(-90);
                driveInches(0, 24);
                break;
        }
    }
}
