package com.kalipsorobotics.intoTheDeep.unusedAutos;

import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.autoActions.FloorToBarHangRoundTrip;
import com.kalipsorobotics.actions.autoActions.InitAuto;
import com.kalipsorobotics.actions.autoActions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.OuttakeTransferReady;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Disabled
public class AutoFloorSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoSpecimen = new KActionSet();
        DriveTrain driveTrain = DriveTrain.getInstance(opModeUtilities);
        Outtake outtake = Outtake.getInstance(opModeUtilities);
        IntakeClaw intakeClaw = IntakeClaw.getInstance(opModeUtilities);
        IMUModule imuModule = IMUModule.getInstance(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = WheelOdometry.getInstance(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        MoveLSAction.setGlobalLinearSlideMaintainTicks(0);
        // Target can always be 0 because Hung said so
        MoveLSAction maintainLS = new MoveLSAction(outtake, 0);
//                MoveLSAction.globalLinearSlideMaintainTicks);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intakeClaw, outtake);
        initAuto.setName("initAuto");

        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(300);
        waitAtStart.setName("waitAtStart");
        redAutoSpecimen.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.setMaxTimeOutMS(4000);
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(FloorToBarHangRoundTrip.SPECIMEN_HANG_POS_X, 350, 0);
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



        //================beginning of push================
        OuttakeTransferReady outtakeTransferReady = new OuttakeTransferReady(outtake);
        outtakeTransferReady.setName("outtakeTransferReady");
        outtakeTransferReady.setDependentActions(specimenHang1);
        redAutoSpecimen.addAction(outtakeTransferReady);

        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependentActions(specimenHang1);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90); //y -500
        moveFloorSamples.addPoint(-1330, -500, -180); //y -475
        moveFloorSamples.addPoint(-1330, -800, -180);// before push
        moveFloorSamples.addPoint(-240, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -775, -180); //y -80
        moveFloorSamples.addPoint(-1330, -1050, -180);// before push
        moveFloorSamples.addPoint(-240, -1050, -180);

//        //third sample to depot
//        moveFloorSamples.addPoint(-1300, -1050, -180);
//        moveFloorSamples.addPoint(-1300, -1175, -180);//before push
//        moveFloorSamples.addPoint(-240, -1175, -180);

        //move back
        moveFloorSamples.addPoint(-530, -1170, -180);//move back out to avoid sample carry
        moveFloorSamples.addPoint(-530,-770,-180); //avoid going into observation zone

        redAutoSpecimen.addAction(moveFloorSamples);
        //==============end of pushing================


        //=============begin of second specimen=================
        FloorToBarHangRoundTrip floorToBarHangRoundTrip2 = new FloorToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,intakeClaw, 375); //400 //375                                  //change
        floorToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        floorToBarHangRoundTrip2.setDependentActions(moveFloorSamples);
        redAutoSpecimen.addAction(floorToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        FloorToBarHangRoundTrip floorToBarHangRoundTrip3 = new FloorToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake,intakeClaw, 450); //400 //375                                  //change
        floorToBarHangRoundTrip3.setName("wallToBarHangRoundTrip2");
        floorToBarHangRoundTrip3.setDependentActions(floorToBarHangRoundTrip2);
        redAutoSpecimen.addAction(floorToBarHangRoundTrip3);
        //===============end of third specimen===========

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
