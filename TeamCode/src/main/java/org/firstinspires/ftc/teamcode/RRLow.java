package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name ="RRLow", group = "A")
public class RRLow extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();


        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.3, 0);
        driveForDistance(0.2, Direction.RIGHT, 0.3, 0);
        driveForDistance(0.17, Direction.FORWARD, 0.3, 0);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.28, Direction.BACKWARD, 0.5, 0);

        GoToHeight(0);

        driveForDistance(1.4, Direction.LEFT, 0.4, 0);


        while(opModeIsActive()) {

        }
    }



}
