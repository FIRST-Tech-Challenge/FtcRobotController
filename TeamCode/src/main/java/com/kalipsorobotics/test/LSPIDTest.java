package com.kalipsorobotics.test;


import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.math.CalculateTickPer;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LSPIDTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        KActionSet set = new KActionSet();
        MoveLSAction test1 = new MoveLSAction(outtake, Outtake.LS_SAMPLE_BASKET_READY_POS);
        WaitAction wait1 = new WaitAction(2000);
        MoveLSAction test2 = new MoveLSAction(outtake, Outtake.LS_SPECIMEN_HANG_READY_MM);
        WaitAction wait2 = new WaitAction(2000);
        MoveLSAction test3 = new MoveLSAction(outtake, Outtake.LS_SPECIMEN_PARK_MM);
        WaitAction wait3 = new WaitAction(2000);
        MoveLSAction test4 = new MoveLSAction(outtake, 0);

        wait1.setDependentActions(test1);
        test2.setDependentActions(wait1);
        wait2.setDependentActions(test2);
        test3.setDependentActions(wait2);
        wait3.setDependentActions(test3);
        test4.setDependentActions(wait3);

        set.addAction(test1, wait1, test2, wait2, test3, wait3, test4);

        waitForStart();
        while (opModeIsActive()) {
            set.updateCheckDone();
//            Log.d("lstest", String.valueOf(test1.getCurrentTicks()));
            Log.d("lstest", String.valueOf(outtake.getLinearSlide1().getCurrentPosition()));
        }
    }
}
