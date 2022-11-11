package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="BRMed", group = "A")
public class BRMed extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();

        waitForStart();

        GoToHeight(1950);
        sleep(500);

        clawRelease();

        sleep(1000);
        GoToHeight(0);

        while (opModeIsActive()) {

        }
    }
}

