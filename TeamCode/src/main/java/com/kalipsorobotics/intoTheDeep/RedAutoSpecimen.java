package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.KServo;
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

        PurePursuitAction moveToFloorSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToFloorSample1.setDependentAction(moveToSpecimenBar);

        PurePursuitAction moveToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall1.setDependentAction(moveToFloorSample1);

        //TODO add linearslide raise

        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar1.setDependentAction(moveToWall1);

        //specimen hang 2

        PurePursuitAction moveToWall2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall2.setDependentAction(moveToFloorSample1);



        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar2.setDependentAction(moveToWall2);

        //specimen hang 3


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
        moveToWall1.addPoint(-80, -600, -180);

        //wall to bar second specimen
        moveWallToBar1.addPoint(-740, 350, 0);

        //bar to wall third specimen
        moveToWall2.addPoint(-80, -600, -180);

        //wall to bar third specimen
        moveWallToBar2.addPoint(-740, 400, 0);



        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            moveToSpecimenBar.updateCheckDone();

            //hang specimen 1
            //raise LS
            //outtake Pivot
            //lower LS

            moveToFloorSample1.updateCheckDone();

            //intake wall

            moveWallToBar1.updateCheckDone();

            moveToWall2.updateCheckDone();

            //outtakePivotWall2.open();

            moveWallToBar2.updateCheckDone();

            //hang specimen 3
        }

    }
}
