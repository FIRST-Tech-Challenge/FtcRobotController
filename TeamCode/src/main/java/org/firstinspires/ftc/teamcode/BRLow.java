package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name ="BRLow", group = "A")
public class BRLow extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.35, 0);
        driveForDistance(0.19, Direction.RIGHT, 0.35, 0);
        driveForDistance(0.15, Direction.FORWARD, 0.35, 0);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.35, 0);

        GoToHeight(0);

        driveForDistance(0.5, Direction.RIGHT, .35, 0);

        while(opModeIsActive()) {

        }
    }



}
