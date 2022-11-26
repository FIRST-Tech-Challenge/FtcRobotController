package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="RRMid", group = "A")
@Disabled
public class RRMid extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        sleep(500);

        waitForStart();

        driveForDistance(0.6, Variables.Direction.RIGHT,0.35,0);
        driveForDistance(0.6, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.33, Variables.Direction.LEFT, 0.35,0);
        goToMid();
        sleep(500);
        driveForDistance(0.15, Variables.Direction.FORWARD,0.35,0);
        clawRelease();
        sleep(200);
        driveForDistance(0.15, Variables.Direction.BACKWARD,0.35,0);
        goToDown();
        sleep(500);
        driveForDistance(0.33, Variables.Direction.RIGHT, 0.35,0);
        driveForDistance(0.6, Variables.Direction.BACKWARD,0.35,0);
        driveForDistance(2.5, Variables.Direction.LEFT,0.35,0);



//        driveForDistance(0.1, Variables.Direction.BACKWARD, 0.35, 0);
//        driveForDistance(1.28, Variables.Direction.LEFT, 0.35, 0);
//        driveForDistance(0.1, Variables.Direction.FORWARD, 0.35, 0);
//
////        clawRelease();
//
//        driveForDistance(0.1, Variables.Direction.BACKWARD, 0.35, 0);
//
////        sleep(1000);
////        GoToHeight(0);
//
//        driveForDistance(1.28, Variables.Direction.RIGHT, 0.35, 0);
//        driveForDistance(1, Variables.Direction.BACKWARD, 0.35, 0);

        while (opModeIsActive()) {

        }
    }
}
