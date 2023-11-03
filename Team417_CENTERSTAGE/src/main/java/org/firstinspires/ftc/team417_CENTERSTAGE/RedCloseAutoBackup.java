package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name="Red Close Auto Backup ")

public class RedCloseAutoBackup extends BaseAutonomous {
    public void runOpMode() {
        initializeAuto();


        waitForStart();

        detectingBlue = false;

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

        while (opModeIsActive()) {

        }

        // move forward from the wall
    }
}
