package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="RBMid", group = "A")
//@Disabled
public class RBMid extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        sleep(500);
        waitForStart();

        GoToHeight(300);

        driveForDistance(0.1, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.65, Variables.Direction.LEFT,0.35,0);
        driveForDistance(0.5, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.30, Variables.Direction.RIGHT, 0.35,0);
        goToMid();
        sleep(500);
        driveForDistance(0.15, Variables.Direction.FORWARD,0.35,0);
        clawRelease();
        sleep(200);
        driveForDistance(0.15, Variables.Direction.BACKWARD,0.35,0);
        goToDown();
        sleep(500);
        driveForDistance(0.30, Variables.Direction.LEFT, 0.35,0);
        driveForDistance(0.6, Variables.Direction.BACKWARD,0.35,0);
        driveForDistance(1.5, Variables.Direction.RIGHT,0.35,0);

        while (opModeIsActive()) {

        }
    }
}
