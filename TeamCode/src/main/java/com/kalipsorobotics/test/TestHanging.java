package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.AutoHangAction;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestHanging extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = new Outtake(opModeUtilities);

        //AutoHangAction autoHangAction = new AutoHangAction(outtake);

        waitForStart();
        while (opModeIsActive()) {
            //autoHangAction.updateCheckDone();
        }
    }
}
