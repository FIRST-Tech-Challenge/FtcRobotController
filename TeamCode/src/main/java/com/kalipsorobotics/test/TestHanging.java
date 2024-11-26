package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.AutoHangAction;
import com.kalipsorobotics.actions.MoveLSAction;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.hang.HangHookAction;
import com.kalipsorobotics.math.CalculateTickInches;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TestHanging extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = new Outtake(opModeUtilities);


        AutoHangAction autoHangAction = new AutoHangAction(outtake);

        //HangHookAction hangHookAction = new HangHookAction(outtake);

        waitForStart();
        while (opModeIsActive()) {
//            moveLSUp.updateCheckDone();
//            waitAction.updateCheckDone();
//            moveDown.updateCheckDone();

            autoHangAction.updateCheckDone();
            //hangHookAction.updateCheckDone();
        }
    }
}
