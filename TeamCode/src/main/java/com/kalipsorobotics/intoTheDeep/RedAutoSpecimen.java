package com.kalipsorobotics.intoTheDeep;

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
public class RedAutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Outtake outtake = new Outtake(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);


        //================begin of first specimen====================
        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToSpecimenBar.setName("moveToSpecimenBar");
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-740, 300, 0);
        HangSpecimenReady hangSpecimenReady1 = new HangSpecimenReady(outtake);
        hangSpecimenReady1.setName("hangSpecimenReady1");
        MoveLSAction lowerSlides1 = new MoveLSAction(outtake, 0);
        lowerSlides1.setName("lowerSlides1");
        lowerSlides1.setDependantActions(moveToSpecimenBar, hangSpecimenReady1);
        //===============end of first specimen===============



        //================beginning of push================
        PurePursuitAction moveFloorSamples = new PurePursuitAction(driveTrain, wheelOdometry);
        moveFloorSamples.setName("moveFloorSamples");
        moveFloorSamples.setDependantActions(lowerSlides1);
        //first sample to depot
        moveFloorSamples.addPoint( -620, -575, -90);
        moveFloorSamples.addPoint(-1330, -600, -180);
        moveFloorSamples.addPoint(-1330, -900, -180);// before push
        moveFloorSamples.addPoint(-130, -800, -180);

        //second sample to depot
        moveFloorSamples.addPoint(-1330, -800, -180);
        moveFloorSamples.addPoint(-1330, -1150, -180);// before push
        moveFloorSamples.addPoint(-130, -1100, -180);
        moveFloorSamples.addPoint(-430, -1100, -180);

        PurePursuitAction moveDepotToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveDepotToWall1.setName("moveDepotToWall1");
        moveDepotToWall1.setDependantActions(moveFloorSamples);
        moveDepotToWall1.addPoint(-80, -600, -180);
        //==============end of pushing================


        //=============begin of second specimen=================
        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar1.setName("moveWallToBar1");
        moveWallToBar1.setDependantActions(moveDepotToWall1);
        //wall to bar second specimen
        moveWallToBar1.addPoint(-740, 350, 0);
        HangSpecimenReady hangSpecimenReady2 = new HangSpecimenReady(outtake);
        hangSpecimenReady2.setName("hangSpecimenReady2");
        hangSpecimenReady2.setDependantActions(moveDepotToWall1);
        MoveLSAction lowerSlides2 = new MoveLSAction(outtake, 100);
        lowerSlides2.setName("lowerSlides2");
        lowerSlides2.setDependantActions(hangSpecimenReady2, moveWallToBar1);
        //===============end of second specimen==============

        //============begin of third================
        PurePursuitAction moveBarToWall2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveBarToWall2.setName("moveBarToWall2");
        moveBarToWall2.setDependantActions(lowerSlides2);
        moveBarToWall2.addPoint(-80, -600, -180);
        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar2.setName("moveWallToBar2");
        moveWallToBar2.setDependantActions(moveBarToWall2);
        moveWallToBar2.addPoint(-740, 400, 0);
        HangSpecimenReady hangSpecimenReady3 = new HangSpecimenReady(outtake);
        hangSpecimenReady3.setName("hangSpecimenReady3");
        hangSpecimenReady3.setDependantActions(moveBarToWall2);
        MoveLSAction lowerSlides3 = new MoveLSAction(outtake, 100);
        lowerSlides3.setName("lowerSlides3");
        lowerSlides3.setDependantActions(hangSpecimenReady3, moveWallToBar2);
        //===============end of third specimen===========




        KActionSet redAutoSpecimen = new KActionSet();

        redAutoSpecimen.addAction(moveToSpecimenBar);
        redAutoSpecimen.addAction(lowerSlides1);
        redAutoSpecimen.addAction(hangSpecimenReady1);
        redAutoSpecimen.addAction(moveFloorSamples);
        redAutoSpecimen.addAction(moveDepotToWall1);
        redAutoSpecimen.addAction(moveWallToBar1);
        redAutoSpecimen.addAction(lowerSlides2);
        redAutoSpecimen.addAction(hangSpecimenReady2);
        redAutoSpecimen.addAction(moveBarToWall2);
        redAutoSpecimen.addAction(moveWallToBar2);
        redAutoSpecimen.addAction(lowerSlides3);
        redAutoSpecimen.addAction(hangSpecimenReady3);


        /*ActionSet redAutoAction = new ActionSet();

        redAutoAction.scheduleSequential(moveToSpecimenBar);
        redAutoAction.scheduleParallel(lowerSlides1);
//        hangSpecimenReady1.setDependantActions(sch, moveToSpecimenBar);
        redAutoAction.scheduleSequential(moveFloorSamples);
        //raise slides
        redAutoAction.scheduleSequential(moveDepotToWall1);
        //intake wall
        //raise slides
        redAutoAction.scheduleSequential(moveWallToBar1);
//        redAutoAction.scheduleSequential(hangSpecimenReady2);
        //raise slides + pivot
        redAutoAction.scheduleSequential(moveBarToWall2);
        redAutoAction.scheduleSequential(moveWallToBar2);
        //raise slides
//        redAutoAction.scheduleSequential(hangSpecimenReady3);
*/
        redAutoSpecimen.printWithDependantActions();
        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            redAutoSpecimen.updateCheckDone();

        }

    }
}
