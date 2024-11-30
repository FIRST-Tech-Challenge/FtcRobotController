package com.kalipsorobotics.test;

import android.util.Log;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.HangSpecimenReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class MechanismTestAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);


        //================begin of first specimen====================
        HangSpecimenReady hangSpecimenReady1 = new HangSpecimenReady(outtake);
        hangSpecimenReady1.setName("hangSpecimenReady1");
        MoveLSAction lowerSlides1 = new MoveLSAction(outtake, 0);
        lowerSlides1.setName("lowerSlides1");
        lowerSlides1.setDependantActions(hangSpecimenReady1);
        //===============end of first specimen===============



        //================beginning of push================
        //==============end of pushing================


        //=============begin of second specimen=================
        HangSpecimenReady hangSpecimenReady2 = new HangSpecimenReady(outtake);
        hangSpecimenReady2.setName("hangSpecimenReady2");
        MoveLSAction lowerSlides2 = new MoveLSAction(outtake, 0);
        lowerSlides2.setName("lowerSlides2");
        lowerSlides2.setDependantActions(hangSpecimenReady2);
        //===============end of second specimen==============

        //============begin of third================
        HangSpecimenReady hangSpecimenReady3 = new HangSpecimenReady(outtake);
        hangSpecimenReady3.setName("hangSpecimenReady3");
        MoveLSAction lowerSlides3 = new MoveLSAction(outtake, 0);
        lowerSlides3.setName("lowerSlides3");
        lowerSlides3.setDependantActions(hangSpecimenReady3);
        //===============end of third specimen===========




        KActionSet redAutoSpecimen = new KActionSet();
        while (opModeInInit()) {
            if (gamepad1.a) {
                redAutoSpecimen.clear();
                redAutoSpecimen.addAction(hangSpecimenReady1);
                telemetry.addLine("done init hang specimen ready");
                telemetry.update();
            }
        }

        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();

        }

    }
}
