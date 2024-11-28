package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.AutoSpecimenHangAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAutoAction;
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

        AutoSpecimenHangAction specimenHang1 = new AutoSpecimenHangAction(outtake);
        specimenHang1.setDependentAction(moveToSpecimenBar);

        PurePursuitAction moveToFloorSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToFloorSample1.setDependentAction(specimenHang1);

        PurePursuitAction moveToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall1.setDependentAction(moveToFloorSample1);

        //TODO add linearslide raise
        OuttakePivotAutoAction intakeWall1 = new OuttakePivotAutoAction(outtake,
                OuttakePivotAutoAction.Position.SPECIMEN);
        intakeWall1.setDependentAction(moveToWall1);

        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar1.setDependentAction(moveToFloorSample1);

        AutoSpecimenHangAction specimenHang2 = new AutoSpecimenHangAction(outtake);
        specimenHang2.setDependentAction(moveWallToBar1);

        PurePursuitAction moveToWall2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveToWall2.setDependentAction(specimenHang2);

        OuttakePivotAutoAction intakeWall2 = new OuttakePivotAutoAction(outtake,
                OuttakePivotAutoAction.Position.SPECIMEN);
        intakeWall2.setDependentAction(moveToWall2);

        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);
        moveWallToBar2.setDependentAction(moveToWall2);

        AutoSpecimenHangAction specimenHang3 = new AutoSpecimenHangAction(outtake);
        specimenHang3.setDependentAction(moveWallToBar2);


        moveToSpecimenBar.setSleep(1000);
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

            specimenHang1.updateCheckDone();

            moveToFloorSample1.updateCheckDone();

            intakeWall1.updateCheckDone();

            moveWallToBar1.updateCheckDone();

            moveToWall2.updateCheckDone();

            intakeWall2.updateCheckDone();

            moveWallToBar2.updateCheckDone();
        }

    }
}
