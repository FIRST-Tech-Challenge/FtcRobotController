package org.firstinspires.ftc.team417_CENTERSTAGE.calibration;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms.BaseAutonomous;

@Autonomous(name="Calibrate Intake")
public class CalibrateIntake extends BaseAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        waitForStart();

        Actions.runBlocking(dropPixelWithIntake());
    }
}
