package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="BBHigh", group = "A")
//@Disabled
public class BBHigh extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        sleep(500);
        waitForStart();

        GoToHeight(300);

        driveForDistance(0.65, Variables.Direction.LEFT,0.35,0);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.28, Variables.Direction.RIGHT, 0.35,0);
        goToHigh();
        driveForDistance(0.25, Variables.Direction.FORWARD,0.2,0);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.15, Variables.Direction.BACKWARD,0.35,0);
        goToDown();
        sleep(500);
        driveForDistance(0.33, Variables.Direction.LEFT, 0.35,0);
        driveForDistance(1.22, Variables.Direction.BACKWARD,0.35,0);
        driveForDistance(1.5, Variables.Direction.RIGHT,0.35,0);

        while (opModeIsActive()) {

        }
    }
}
