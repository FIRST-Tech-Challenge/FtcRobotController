package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Blue Close Auto Backup ")

public class BlueCloseAutoBackup extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();
        driveInches(24,0);

        detectingBlue = true;

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}
