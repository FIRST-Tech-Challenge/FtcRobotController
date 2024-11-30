package com.kalipsorobotics.intoTheDeep;

import android.util.Log;

import com.kalipsorobotics.actions.InitAuto;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.HangSpecimenReady;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.WaitLowerSlides;
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
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        Intake intake = new Intake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        InitAuto initAuto = new InitAuto(intake);

        //================begin of first specimen====================
        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-785, 300, 0);
        HangSpecimenReady hangSpecimenReady1 = new HangSpecimenReady(outtake);
        hangSpecimenReady1.setName("hangSpecimenReady1");
        MoveLSAction lowerSlidesHalf1 = new MoveLSAction(outtake, 200);
        lowerSlidesHalf1.setName("lowerSlidesHalf1");
        lowerSlidesHalf1.setDependantActions(hangSpecimenReady1, moveToSpecimenBar);
        WaitLowerSlides waitLowerSlidesZero1 = new WaitLowerSlides(outtake);
        waitLowerSlidesZero1.setName("waitLowerSlidesZero1");
        waitLowerSlidesZero1.setDependantActions(lowerSlidesHalf1);
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
        moveFloorSamples.addPoint(-130, -1100, -180);
        moveFloorSamples.addPoint(-430, -1100, -180);

        PurePursuitAction moveDepotToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveDepotToWall1.setName("moveDepotToWall1");
        moveDepotToWall1.setDependantActions(moveFloorSamples);
        moveDepotToWall1.addPoint(-205, -600, -180);
        moveDepotToWall1.addPoint(-55, -600, -180);

        WaitAction waitAtWall1 = new WaitAction(1000);
        waitAtWall1.setName("waitAtWall1");
        waitAtWall1.setDependantActions(moveDepotToWall1);
        //==============end of pushing================


        //=============begin of second specimen=================
        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar1.setName("moveWallToBar1");
        moveWallToBar1.setDependantActions(waitAtWall1);
        moveWallToBar1.addPoint(-785, 250, 0);
        HangSpecimenReady hangSpecimenReady2 = new HangSpecimenReady(outtake);
        hangSpecimenReady2.setName("hangSpecimenReady2");
        hangSpecimenReady2.setDependantActions(waitAtWall1);
        MoveLSAction lowerSlidesHalf2 = new MoveLSAction(outtake, 100);
        lowerSlidesHalf2.setName("lowerSlidesHalf2");
        lowerSlidesHalf2.setDependantActions(hangSpecimenReady2, moveWallToBar1);
        WaitLowerSlides waitLowerSlidesZero2 = new WaitLowerSlides(outtake);
        waitLowerSlidesZero2.setName("waitLowerSlidesZero2");
        waitLowerSlidesZero2.setDependantActions(lowerSlidesHalf2);
        //===============end of second specimen==============

        //============begin of third================
        PurePursuitAction moveBarToWall2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveBarToWall2.setName("moveBarToWall2");
        //moveBarToWall2.setDependantActions(lowerSlidesHalf2);
        moveBarToWall2.setDependantActions(moveWallToBar1);
        moveBarToWall2.addPoint(-80, -600, -180);
        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar2.setName("moveWallToBar2");
        moveWallToBar2.setDependantActions(moveBarToWall2);
        moveWallToBar2.addPoint(-740, 400, 0);
        //HangSpecimenReady hangSpecimenReady3 = new HangSpecimenReady(outtake);
        //hangSpecimenReady3.setName("hangSpecimenReady3");
        //hangSpecimenReady3.setDependantActions(moveBarToWall2);
        //MoveLSAction lowerSlides3 = new MoveLSAction(outtake, 100);
        //lowerSlides3.setName("lowerSlides3");
        //lowerSlides3.setDependantActions(hangSpecimenReady3, moveWallToBar2);
        //===============end of third specimen===========




        KActionSet redAutoSpecimen = new KActionSet();

        //===========Specimen1==============
        redAutoSpecimen.addAction(moveToSpecimenBar);
        redAutoSpecimen.addAction(hangSpecimenReady1);
        redAutoSpecimen.addAction(lowerSlidesHalf1);
        redAutoSpecimen.addAction(waitLowerSlidesZero1);
        //===============PushSample==========
        redAutoSpecimen.addAction(moveFloorSamples);
        //===========Specimen2==============
        redAutoSpecimen.addAction(moveDepotToWall1);
        redAutoSpecimen.addAction(waitAtWall1);
        redAutoSpecimen.addAction(moveWallToBar1);
        redAutoSpecimen.addAction(hangSpecimenReady2);
        redAutoSpecimen.addAction(lowerSlidesHalf2);
        //===========Specimen3==============
        //redAutoSpecimen.addAction(moveBarToWall2);
        //redAutoSpecimen.addAction(moveWallToBar2);
        //redAutoSpecimen.addAction(hangSpecimenReady3);
        //redAutoSpecimen.addAction(lowerSlides3);

        initAuto.updateCheckDone();

        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();


        }

    }
}
