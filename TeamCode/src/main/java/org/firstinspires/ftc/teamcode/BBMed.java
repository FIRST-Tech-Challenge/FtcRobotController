package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name ="BBMed", group = "A")
public class BBMed extends DriveMethods{
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

