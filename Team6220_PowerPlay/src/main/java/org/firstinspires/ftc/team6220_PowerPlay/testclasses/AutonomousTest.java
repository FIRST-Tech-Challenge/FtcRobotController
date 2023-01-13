package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import java.util.List;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Test")
public class AutonomousTest extends BaseAutonomous {
    double motorPower;
    double x;
    double width = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize
        initialize();

        // detects signal and replaces wait for start
        int signal = detectSignal();

        // grab cone
        servoGrabber.setPosition(Constants.GRABBER_CLOSE_POSITION);

        // raise slides to stow position
        driveSlidesAutonomous(Constants.SLIDE_STOW);

        // detect signal
        detectSignal();

        // drive forward 54 inches
        // todo - 54?
        driveInches(0, 54);

        // turn to -45 degrees
        turnToAngle(-45);
    }
}
