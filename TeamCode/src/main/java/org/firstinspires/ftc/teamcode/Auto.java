package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Auto {
    private boolean isAutoStarted = false;
    private boolean isShotStarted = false;
    private final MecanumDriveTrain driveTrain;
    private final Launcher launcher;
    private final ElapsedTime shotTimer = new ElapsedTime();
    private final ElapsedTime driveTimer = new ElapsedTime();

    public Auto(MecanumDriveTrain driveTrain, Launcher launcher) {
        this.driveTrain = driveTrain;
        this.launcher = launcher;
    }

    public void doAutoBlueGoal() {
        launcher.shootWithTime(1, 7000);
        driveTrain.driveWithTime(-1,-1, 0, 1, 1000);
    }


    public void doAutoRedGoal() {
       launcher.shootWithTime(1, 7000);
       driveTrain.driveWithTime(-1,1, 0, 1, 1000);
    }

    public void doAutoBlueWall() {
        driveTrain.driveWithTime(1,0,0,0.3,200);
        driveTrain.driveWithTime(0, 0, -1, 0.3, 450);
        launcher.shootWithTime(3, 7000);
        driveTrain.driveWithTime(0,0,1,0.3,450);
        driveTrain.driveWithTime(0, -1, 0, 1, 750);
    }
    public void doAutoRedWall() {
        driveTrain.driveWithTime(1,0,0,0.3,200);
        driveTrain.driveWithTime(0,0,1,0.3,450);
        launcher.shootWithTime(3, 7000);
        driveTrain.driveWithTime(0, 0, -1, 0.3, 450);
        driveTrain.driveWithTime(0, 1, 0, 1, 750);
    }

    private void farShot(int timeout_ms) {
        if (isShotStarted == false) {
            shotTimer.reset();
            isShotStarted = true;
        }
        while (shotTimer.milliseconds() < timeout_ms) {
            launcher.farShot();
        }
    }

}
