package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;


/**
 * A OpMode to test Odometry turning capabilities
 */
@Disabled
@Autonomous(name = "OdMove")
public class odMove extends AutoObjDetectionTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        tfod.shutdown();
        vuforia.close();
        initOdometryServos();
        gps.setPos(6, 111, 180);
        driveSystem.setTurnWhileStrafe(true);
        telemetry.addData("Initialization Status", "complete");
        telemetry.update();
        BarcodePositions Pos = BarcodePositions.Left;


        waitForStart();
        driveSystem.debugOn();


        driveSystem.moveToPosition(26, 82.5, 272, .5, new DistanceTimeoutWarning(500));


        dropOffFreight(Pos);

        Thread.sleep(2000);


        telemetry.addData("done", "");
        telemetry.update();

        //driveSystem.stopAll();

    }

}
