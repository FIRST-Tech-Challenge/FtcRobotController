package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.enums.BarcodePositions;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

/**
 * A Autonomous to test our machine learning model
 */
@Disabled
@Autonomous(name = "MLTest")
public class MLTest extends AutoObjDetectionTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();

        BarcodePositions Pos = BarcodePositions.Right;

        while (!isStarted()) {
            Pos = this.getAverageOfMarker(10, 100);
            telemetry.addData("Position", Pos);
            telemetry.update();
        }
        waitForStart();

        tfod.shutdown();
        vuforia.close();

        System.gc();


        switch (Pos) {
            case NotSeen:
                telemetry.addData("position", " is left");
                // write movements underneath this
                break;
            case Right:
                telemetry.addData("position", " is right");

                // write movements underneath this
                break;
            case Left:
                telemetry.addData("position", "is center");

                // write movements underneath this
                break;


        }
        telemetry.update();

        //Essentially tells the robot controller to ignore this thread until stop is pressed
        Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
        while (opModeIsActive() && !isStopRequested()) {
            Thread.yield();
        }

    }
}
