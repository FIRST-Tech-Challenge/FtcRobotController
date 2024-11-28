package com.kalipsorobotics.intoTheDeep;

import com.kalipsorobotics.actions.PurePursuitAction;
import com.kalipsorobotics.actions.WaitAction;
import com.kalipsorobotics.actions.outtake.OuttakePivotAction;
import com.kalipsorobotics.actions.outtake.SpecimenHangAction;
import com.kalipsorobotics.localization.WheelOdometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.modules.IMUModule;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
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
        PurePursuitAction moveToFloorSample2 = new PurePursuitAction(driveTrain, wheelOdometry);
        PurePursuitAction moveToFloorSample3 = new PurePursuitAction(driveTrain, wheelOdometry);

        moveToSpecimenBar.setSleep(1000);
        moveToSpecimenBar.addPoint(0, 0, 0);
        moveToSpecimenBar.addPoint(-740, 300, 0);

        //first sample to depot
        moveToFloorSample1.addPoint( -620, -600, -80);
        moveToFloorSample1.addPoint(-1330, -600, -80);
        moveToFloorSample1.addPoint(-1330, -800, -70);
        moveToFloorSample1.addPoint(-130, -800, -80);
        //second sample to depot
        moveToFloorSample1.addPoint(-1330, -800, -80);
        moveToFloorSample1.addPoint(-1330, -1000, -80);
        moveToFloorSample1.addPoint(-130, -1100, -80);
        //sample to wall
        moveToFloorSample1.addPoint(-80, -500, -180);


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

        }

    }
}
