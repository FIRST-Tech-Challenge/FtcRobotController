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

        MoveLSAction moveLSUp = new MoveLSAction(30, outtake);

        WaitAction waitAction = new WaitAction(2);
        waitAction.setDependentAction(moveLSUp);

        MoveLSAction moveDown = new MoveLSAction(-12, outtake, 0.01);
        moveDown.setDependentAction(waitAction);

        waitForStart();
        while (opModeIsActive()) {
            moveLSUp.updateCheckDone();
            waitAction.updateCheckDone();
            moveDown.updateCheckDone();
        }
    }
}
