package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.KServoAutoAction;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.WallToBarHangRoundTrip;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenWallReady;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeClawAction;
import com.kalipsorobotics.actions.outtake.teleopActions.OuttakeSlideAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Intake;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class RedAutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        KActionSet redAutoSpecimen = new KActionSet();
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);

        MoveLSAction maintainLS = new MoveLSAction(outtake, MoveLSAction.globalLinearSlideMaintainPos);
        maintainLS.setName("maintainLS");

        InitAuto initAuto = new InitAuto(intake, outtake);
        initAuto.setName("initAuto");


        //================begin of first specimen====================
        WaitAction waitAtStart = new WaitAction(500);
        waitAtStart.setName("waitAtStart");
        redAutoSpecimen.addAction(waitAtStart);

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-785, 300, 0);
        moveToSpecimenBar.setDependantActions(waitAtStart);
        redAutoSpecimen.addAction(moveToSpecimenBar);

        SpecimenHangReady specimenHangReady1 = new SpecimenHangReady(outtake);
        specimenHangReady1.setName("hangSpecimenReady1");
        redAutoSpecimen.addAction(specimenHangReady1);

        MoveLSAction lowerSlidesHalf1 = new MoveLSAction(outtake, 200);
        lowerSlidesHalf1.setName("lowerSlidesHalf1");
        lowerSlidesHalf1.setDependantActions(specimenHangReady1, moveToSpecimenBar);
        redAutoSpecimen.addAction(lowerSlidesHalf1);

        KServoAutoAction openClaw = new KServoAutoAction(outtake.getOuttakeClawServo(),
                OuttakeClawAction.OUTTAKE_CLAW_OPEN_POS);
        openClaw.setName("openClaw");
        openClaw.setDependantActions(lowerSlidesHalf1);
        redAutoSpecimen.addAction(openClaw);

        SpecimenWallReady specimenWallReady = new SpecimenWallReady(outtake);
        specimenWallReady.setName("specimenWallReady");
        specimenWallReady.setDependantActions(lowerSlidesHalf1);
        redAutoSpecimen.addAction(specimenWallReady);

        //===============end of first specimen===============



        //================beginning of push================
        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependantActions(lowerSlidesHalf1);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -475, -90);
        moveFloorSamples.addPoint(-1330, -500, -180);
        moveFloorSamples.addPoint(-1330, -800, -180);// before push
        moveFloorSamples.addPoint(-130, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -800, -180);
        moveFloorSamples.addPoint(-1330, -1050, -180);// before push
        moveFloorSamples.addPoint(-130, -1050, -180);

        //third sample to depot
        moveFloorSamples.addPoint(-1300, -1050, -180);
        moveFloorSamples.addPoint(-1300, -1175, -180);//before push
        moveFloorSamples.addPoint(-130, -1175, -180);
        moveFloorSamples.addPoint(-430, -1170, -180);//move back out to avoid sample carry

        //wall to depot for specimen
        moveFloorSamples.addPoint(-205, -600, -180);
        moveFloorSamples.addPoint(-55, -600, -180);
        redAutoSpecimen.addAction(moveFloorSamples);
        //==============end of pushing================


        //=============begin of second specimen=================
        WallToBarHangRoundTrip wallToBarHangRoundTrip2 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 375);
        wallToBarHangRoundTrip2.setName("wallToBarHangRoundTrip2");
        wallToBarHangRoundTrip2.setDependantActions(moveFloorSamples);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip2);
        //===============end of second specimen==============

        //============begin of third================
        WallToBarHangRoundTrip wallToBarHangRoundTrip3 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 450);
        wallToBarHangRoundTrip3.setName("wallToBarHangRoundTrip3");
        wallToBarHangRoundTrip3.setDependantActions(wallToBarHangRoundTrip2);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip3);
        //===============end of third specimen===========

        //===============start of fourth specimen==============
        WallToBarHangRoundTrip wallToBarHangRoundTrip4 = new WallToBarHangRoundTrip(driveTrain, wheelOdometry,
                outtake, 525);
        wallToBarHangRoundTrip4.setName("wallToBarHangRoundTrip4");
        wallToBarHangRoundTrip4.setDependantActions(wallToBarHangRoundTrip3);
        redAutoSpecimen.addAction(wallToBarHangRoundTrip4);
        //================end of specimen 4================



        initAuto.updateCheckDone();

        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            maintainLS.update();

            redAutoSpecimen.updateCheckDone();

        }

    }
}
