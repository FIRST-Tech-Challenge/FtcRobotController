package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.SpecimenSweepingRoundTrip;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.intake.IntakeTransferReady;
import com.kalipsorobotics.actions.intake.SampleIntakeAction;
import com.kalipsorobotics.actions.intake.SampleIntakeReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class AutoSpecimenSweeping extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoSpecimen = new KActionSet();
        DriveTrain.setInstanceNull();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);

        Outtake.setInstanceNull();
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        IntakeClaw.setInstanceNull();
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);

        IMUModule.setInstanceNull();
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);

        WheelOdometry.setInstanceNull();
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(300);
        waitAtStart.setName("waitAtStart");
        redAutoSpecimen.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.setMaxTimeOutMS(3000);
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, 300, 0); //y 300
        moveToSpecimenBar.setDependentActions(waitAtStart);
        redAutoSpecimen.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        redAutoSpecimen.addAction(specimenHangReady1);

        SpecimenHang specimenHang1 = new SpecimenHang(outtake);
        specimenHang1.setName("specimenHang1");
        specimenHang1.setDependentActions(specimenHangReady1, moveToSpecimenBar);
        redAutoSpecimen.addAction(specimenHang1);
        //===============end of first specimen===============

        //================beginning of sweeping================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(specimenHang1);
        redAutoSpecimen.addAction(outtakeTransferReady);

        // First Sample Sweep
        SpecimenSweepingRoundTrip firstSample = new SpecimenSweepingRoundTrip(driveTrain, intakeClaw, wheelOdometry,-325);
        firstSample.setName("firstSample");
        redAutoSpecimen.addAction(firstSample);
        firstSample.setDependentActions(specimenHang1);


        // Second Sample Sweep
        SpecimenSweepingRoundTrip secondSample = new SpecimenSweepingRoundTrip(driveTrain, intakeClaw, wheelOdometry,-570);
        secondSample.setName("secondSample");
        secondSample.setDependentActions(firstSample);
        redAutoSpecimen.addAction(secondSample);

        // Third Sample Grab
        SpecimenSweepingRoundTrip thirdSample = new SpecimenSweepingRoundTrip(driveTrain, intakeClaw, wheelOdometry,-790);
        thirdSample.setName("secondSample");
        thirdSample.setDependentActions(secondSample);
        redAutoSpecimen.addAction(thirdSample);

        // END OF ALL SAMPLES

        IntakeTransferReady intakeIn = new IntakeTransferReady(intakeClaw);
        intakeIn.setName("intakeIn");
        intakeIn.setDependentActions(thirdSample);
        redAutoSpecimen.addAction(intakeIn);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependentActions(secondSample);
        redAutoSpecimen.addAction(specimenWallReady);

        PurePursuitAction moveToDepot = new PurePursuitAction(driveTrain,wheelOdometry, 1.0/2200.0);
        moveToDepot.setName("moveToDepot");
        moveToDepot.setDependentActions(thirdSample); //CHANGE
        //to depot for specimen
        //moveToDepot.addPoint(-380, -1050, -180); //-380, -615
        moveToDepot.addPoint(WallToBarHangRoundTrip.WALL_PICKUP_X, -1065, -180); //-130, -615
        redAutoSpecimen.addAction(moveToDepot);


        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 400); //400 //375
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setDependentActions(moveToDepot, specimenWallReady, intakeIn);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 500); //500 //450
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setDependentActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);


        //===============end of third specimen===========

        initAuto.update();

        redAutoSpecimen.printWithDependentActions();

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            maintainLS.setIsDone(false);
            maintainLS.setTargetTicks(MoveLSAction.getGlobalLinearSlideMaintainTicks());
            maintainLS.updateCheckDone();

            redAutoSpecimen.updateCheckDone();

        }
    }
}


/*
wall to bar hang #1 specimen

-> Start Roundtrip but change y ~ -250 each time
pure pursuit to sample 1 to depot (-550, -200, heading [-120]) + intake ready
intake sample 1
intake ready (claw closed)
pure pursuit heading [-45]
open claw

pure pursuit to sample 2 to depot (-550, -400, heading [-135]) + intake ready





 */



