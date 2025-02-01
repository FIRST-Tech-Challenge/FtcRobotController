package com.kalipsorobotics.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.kalipsorobotics.actions.KActionSet;
import com.kalipsorobotics.actions.autoActions.KServoAutoAction;
import com.kalipsorobotics.actions.outtake.CloseWhenDetectDistanceAction;
import com.kalipsorobotics.actions.outtake.MoveLSAction;
import com.kalipsorobotics.actions.outtake.SpecimenHang;
import com.kalipsorobotics.actions.outtake.SpecimenHangReady;
import com.kalipsorobotics.modules.IntakeClaw;
import com.kalipsorobotics.modules.Outtake;
import com.kalipsorobotics.modules.RevDistance;
import com.kalipsorobotics.utilities.KServo;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.K;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class RevDistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        Rev2mDistanceSensor revDistance = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance");
        Rev2mDistanceSensor revDistance2 = hardwareMap.get(Rev2mDistanceSensor.class, "revDistance2");
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        Outtake outtake = Outtake.getInstance(opModeUtilities);

        KActionSet darrenchan = new KActionSet();
//
//
//        SpecimenHangReady specimenHangReady = new SpecimenHangReady(outtake);
//        specimenHangReady.setName("specimenHangReady");
//        darrenchan.addAction(specimenHangReady);
//
//        KServoAutoAction closeClaw = new KServoAutoAction(outtake.getOuttakeClaw(), Outtake.OUTTAKE_CLAW_CLOSE);
//        closeClaw.setName("closeClaw");
//        darrenchan.addAction(closeClaw);
//
//        CloseWhenDetectDistanceAction checkBarDistance = new CloseWhenDetectDistanceAction(revDistance2,145);
//        checkBarDistance.setName("checkBarDistance");
//        darrenchan.addAction(checkBarDistance);
//
//
//        SpecimenHang specimenHang = new SpecimenHang(outtake);
//        specimenHang.setName("specimenHang");
//        darrenchan.addAction(specimenHang);
//        specimenHang.setDependentActions(checkBarDistance);



        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("claw", revDistance.getDistance(DistanceUnit.MM));
            telemetry.addData("bottom", revDistance2.getDistance(DistanceUnit.MM));
//
            darrenchan.updateCheckDone();
            telemetry.update();
        }
    }
}
