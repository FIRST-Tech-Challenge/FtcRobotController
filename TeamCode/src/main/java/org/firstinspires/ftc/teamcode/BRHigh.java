package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="BRHigh", group = "A")
//@Disabled
public class BRHigh extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        sleep(500);
        waitForStart();

        GoToHeight(300);

        driveForDistance(0.1, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.62, Variables.Direction.RIGHT,0.35,0);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35,0);
        driveForDistance(0.45, Variables.Direction.LEFT, 0.35,0);
        goToHigh();
        driveForDistance(0.145, Variables.Direction.FORWARD,0.2,0);
        sleep(1000);
        clawRelease();
        sleep(200);
        driveForDistance(0.135, Variables.Direction.BACKWARD,0.35,0);
        goToDown();
        sleep(500);
        driveForDistance(0.35, Variables.Direction.RIGHT, 0.35,0);
        driveForDistance(1.22, Variables.Direction.BACKWARD,0.35,0);
        driveForDistance(1.55, Variables.Direction.LEFT,0.35,120);

        while (opModeIsActive()) {

        }
    }
}
