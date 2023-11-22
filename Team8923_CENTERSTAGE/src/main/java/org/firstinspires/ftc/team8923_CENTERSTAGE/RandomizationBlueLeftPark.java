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
                break;
        }
    }
}
