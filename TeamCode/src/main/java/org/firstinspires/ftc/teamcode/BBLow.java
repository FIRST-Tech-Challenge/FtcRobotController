package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name ="BBLow", group = "A")
public class BBLow extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();


        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.2, Direction.LEFT, 0.35, 0);
        driveForDistance(0.3, Direction.FORWARD, 0.35, 0);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.5, 0);

        GoToHeight(0);

        driveForDistance(1.4, Direction.RIGHT, 0.5, 0);

        while(opModeIsActive()) {

        }
    }



}
