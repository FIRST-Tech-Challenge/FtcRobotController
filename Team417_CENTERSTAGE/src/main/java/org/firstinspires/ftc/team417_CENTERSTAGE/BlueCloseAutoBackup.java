package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Blue Close Auto Backup ")

public class BlueCloseAutoBackup extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();
        waitForStart();

        detectingBlue = true;

<<<<<<< HEAD
        switch (detectTeamProp()) {
            case LEFT:
                telemetry.addData("Side", "Left");
                break;
            case CENTER:
                telemetry.addData("Side", "Center");
                break;
            case RIGHT:
                telemetry.addData("Side", "Right");
                break;
            default:
                telemetry.addData("Side", "Unsure");
        }
        telemetry.update();

        while (opModeIsActive()) {

        }
=======
        while (opModeIsActive()) {

        }

>>>>>>> 061aedadfc98461096b0e2f5f61b2ae85397ee5e
        // move forward from the wall
    }
}
