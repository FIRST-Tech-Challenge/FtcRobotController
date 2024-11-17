package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.MoveLSAction;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
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

        MoveLSAction moveLSUp = new MoveLSAction(CalculateTickInches.inchToTicksLS(30), outtake);
        WaitAction waitAction = new WaitAction(5);
        waitAction.setDependentAction(moveLSUp);
        MoveLSAction moveLSDown = new MoveLSAction(CalculateTickInches.inchToTicksLS(26), outtake);
        moveLSDown.setDependentAction(waitAction);

        waitForStart();
        while (opModeIsActive()) {
            moveLSUp.updateCheckDone();
            waitAction.updateCheckDone();
            moveLSDown.updateCheckDone();
        }
    }
}
