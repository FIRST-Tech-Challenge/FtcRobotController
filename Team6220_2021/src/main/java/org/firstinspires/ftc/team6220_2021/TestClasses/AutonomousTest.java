package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_2021.MasterAutonomous;
import org.firstinspires.ftc.team6220_2021.ResourceClasses.Constants;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                servoArm.setPosition(0.4);
                motorArm.setTargetPosition(300);
                driveInches(6, Constants.MINIMUM_DRIVE_POWER, false);
                pauseMillis(125);
                turnDegrees(-60);
                pauseMillis(125);
                driveInches(48 / Math.sqrt(3), Constants.MINIMUM_DRIVE_POWER, false);
                pauseMillis(125);
                turnDegrees(60);
                pauseMillis(125);
                // move forward if necessary
                pauseMillis(500);
                servoGrabber.setPosition(0.34);
                pauseMillis(500);
                driveInches(40, Constants.MINIMUM_DRIVE_POWER, true);
                motorArm.setTargetPosition(1100);
                driveInches(4, Constants.MINIMUM_DRIVE_POWER, false);
                pauseMillis(125);
                turnDegrees(90);
                pauseMillis(125);
                driveInches(50, Constants.MINIMUM_DRIVE_POWER, false);
                motorLeftDuck.setPower(0.6);
                pauseMillis(2000);
                motorLeftDuck.setPower(0.0);
                driveInches(4, Constants.MINIMUM_DRIVE_POWER, true);
                pauseMillis(125);
                turnDegrees(180);
                pauseMillis(125);
                driveInches(96, 0.75, false);
                break;
            }
        }
    }
}