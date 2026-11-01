package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

/* Bench test: drive forward 3 inches, then back 3 inches. */
@Autonomous(name = "Test Forward Back", group = "Test")
public class AutoTestForwardBack extends AutoBase {

    @Override
    protected void runRoutine() {
        driveInches(3, -RobotConstants.AUTO_POWER);
        driveInches(3, RobotConstants.AUTO_POWER);
    }
}