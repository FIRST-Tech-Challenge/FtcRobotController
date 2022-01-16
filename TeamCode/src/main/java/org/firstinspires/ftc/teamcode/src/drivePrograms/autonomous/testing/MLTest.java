package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.utills.AutoObjDetectionTemplate;

/**
 * A Autonomous to test our machine learning model
 */
@Disabled
@Autonomous(name = "MLTest")
public class MLTest extends AutoObjDetectionTemplate {
    @Override
    public void runOpMode() throws InterruptedException {

        this.initAll();
        this.initVuforia();
        this.initTfod();

        this.activateTF();
        MarkerPosition Pos = MarkerPosition.Right;

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
