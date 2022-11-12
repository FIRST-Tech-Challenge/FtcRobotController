package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="RRMed", group = "A")
public class RRMed extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

//        clawClamp();
//        GoToHeight(1950);
//        sleep(500);

        waitForStart();

        driveForDistance(0.1, Variables.Direction.BACKWARD, 0.35, 0);
        driveForDistance(1, Variables.Direction.LEFT, 0.35, 0);
        driveForDistance(0.1, Variables.Direction.FORWARD, 0.35, 0);

//        clawRelease();

        driveForDistance(0.1, Variables.Direction.BACKWARD, 0.35, 0);

//        sleep(1000);
//        GoToHeight(0);

        driveForDistance(1, Variables.Direction.RIGHT, 0.35, 0);
        driveForDistance(1, Variables.Direction.BACKWARD, 0.35, 0);

        while (opModeIsActive()) {

        }
    }
}
