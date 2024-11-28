package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.outtake.SpecimenHangAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class RedAutoSpecimen extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        IMUModule imuModule = new IMUModule(opModeUtilities);
        sleep(1000);
        WheelOdometry wheelOdometry = new WheelOdometry(opModeUtilities, driveTrain, imuModule, 0, 0, 0);
        PurePursuitAction moveToSpecimenBar = new PurePursuitAction(driveTrain, wheelOdometry);
        SpecimenHangAction specimenHangAction1 = new SpecimenHangAction();
        SpecimenHangAction specimenHangAction2 = new SpecimenHangAction();
        SpecimenHangAction specimenHangAction3 = new SpecimenHangAction();
        PurePursuitAction moveToFloorSample1 = new PurePursuitAction(driveTrain, wheelOdometry);
        PurePursuitAction moveWallToBar1 = new PurePursuitAction(driveTrain, wheelOdometry);
        PurePursuitAction moveBarToWall1 = new PurePursuitAction(driveTrain, wheelOdometry);
        PurePursuitAction moveWallToBar2 = new PurePursuitAction(driveTrain, wheelOdometry);

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
        moveToFloorSample1.addPoint(-80, -600, -180);

        //wall to bar second specimen
        moveWallToBar1.addPoint(-740, 350, 0);

        //bar to wall third specimen
        moveBarToWall1.addPoint(-80, -600, -180);

        //wall to bar third specimen
        moveWallToBar2.addPoint(-740, 400, 0);

        //

        waitForStart();
        while (opModeIsActive()) {

            wheelOdometry.updatePosition();

            moveToSpecimenBar.update();

            if (moveToSpecimenBar.checkDoneCondition()) {
                specimenHangAction1.update();
            }

            if (moveToSpecimenBar.checkDoneCondition()/*specimenHangAction1.checkDoneCondition()*/) {
                moveToFloorSample1.update();
            }

            if (moveToFloorSample1.checkDoneCondition()) {
                moveWallToBar1.update();
            }

            if (moveWallToBar1.checkDoneCondition()) {
                moveBarToWall1.update();
            }

            if (moveBarToWall1.checkDoneCondition()) {
                moveWallToBar2.update();
            }

            if (moveWallToBar2.checkDoneCondition()) {
                break;
            }
        }

    }
}
