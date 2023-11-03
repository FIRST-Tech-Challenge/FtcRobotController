package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Red Close Auto Backup ")

public class RedCloseAutoBackup extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();

        waitForStart();

        detectingBlue = false;

<<<<<<< HEAD
        sleep(3000);

        switch (detectTeamProp()) {
            case INITIALIZED:
                telemetry.addData("Side", "Just Initialized");
                break;
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

=======
>>>>>>> 061aedadfc98461096b0e2f5f61b2ae85397ee5e
        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}
