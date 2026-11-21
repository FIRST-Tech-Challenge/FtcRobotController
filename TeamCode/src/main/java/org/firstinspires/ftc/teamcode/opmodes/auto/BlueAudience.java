package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.LauncherConfig;
import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@Autonomous(name = "BlueAudience", group = "Competition")
public class BlueAudience extends AutoBase {

    @Override
    protected void runRoutine() {
        // give the flywheel time to get up to speed
        robot.launcher.spinUp();


        // only shoot if the hive is tipped
        if (scanForTip(RobotConstants.TAG_BLUE_AUDIENCE_MIN, RobotConstants.TAG_BLUE_AUDIENCE_MAX)) {
            shoot();
        }

        leaveAndPark();
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(24, p);
        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
