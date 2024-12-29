package com.kalipsorobotics.test;

import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class TestHanging extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        //AutoHangAction autoHangAction = new AutoHangAction(outtake);

        waitForStart();
        while (opModeIsActive()) {
            //autoHangAction.updateCheckDone();
        }
    }
}
