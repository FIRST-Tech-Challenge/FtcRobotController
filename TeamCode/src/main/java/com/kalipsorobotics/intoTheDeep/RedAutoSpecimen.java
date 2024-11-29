package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.ActionSet;
import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.HangSpecimenActionSet;
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

        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);

        //hang specimen
        HangSpecimenActionSet hangSpecimen1 = new HangSpecimenActionSet(outtake);

        PurePursuitAction moveToFloorSample1 = new PurePursuitAction(driveTrain, wheelOdometry);

        PurePursuitAction moveBarToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);

        HangSpecimenActionSet hangSpecimen2 = new HangSpecimenActionSet(outtake);

        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);

        //specimen hang 2

        PurePursuitAction moveBarToWall2 = new PurePursuitAction(driveTrain, wheelOdometry);

        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);

        //specimen hang 3
        HangSpecimenActionSet hangSpecimen3 = new HangSpecimenActionSet(outtake);


        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-740, 300, 0);

        //first sample to depot
        moveToFloorSample1.addPoint( -620, -575, -90);
        moveToFloorSample1.addPoint(-1330, -600, -180);
        moveToFloorSample1.addPoint(-1330, -900, -180);// before push
        moveToFloorSample1.addPoint(-130, -800, -180);

        //second sample to depot
        moveToFloorSample1.addPoint(-1330, -800, -180);
        moveToFloorSample1.addPoint(-1330, -1150, -180);// before push
        moveToFloorSample1.addPoint(-130, -1100, -180);
        moveToFloorSample1.addPoint(-430, -1100, -180);

        //sample to wall
        moveBarToWall1.addPoint(-80, -600, -180);

        //wall to bar second specimen
        moveWallToBar1.addPoint(-740, 350, 0);

        //bar to wall third specimen
        moveBarToWall2.addPoint(-80, -600, -180);

        //wall to bar third specimen
        moveWallToBar2.addPoint(-740, 400, 0);

        ActionSet redAutoAction = new ActionSet();

        redAutoAction.scheduleSequential(moveToSpecimenBar);
        //raise slides
        redAutoAction.scheduleSequential(hangSpecimen1);
        redAutoAction.scheduleSequential(moveToFloorSample1);
        //raise slides
        redAutoAction.scheduleSequential(moveBarToWall1);
        //intake wall
        //raise slides
        redAutoAction.scheduleSequential(moveWallToBar1);
        redAutoAction.scheduleSequential(hangSpecimen2);
        //raise slides + pivot
        redAutoAction.scheduleSequential(moveBarToWall2);
        redAutoAction.scheduleSequential(moveWallToBar2);
        //raise slides
        redAutoAction.scheduleSequential(hangSpecimen3);

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();
            redAutoAction.updateCheckDone();

        }

    }
}
