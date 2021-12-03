package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.MiscUtills;


@Autonomous(name = "MLTest")
public class MLTest extends AutoObjDetectionTemplate {
    @Override
    public void runOpMode() throws InterruptedException {

        this.initAll();
        this.initVuforia();
        this.initTfod();

        this.activate();
        MarkerPosition Pos = MarkerPosition.Left;

        try {
            Pos = this.getAverageOfMarker(10, 100);
        } catch (Exception e) {

            telemetry.addData("Error", MiscUtills.getStackTraceAsString(e));
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) {
            }

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
        while (opModeIsActive() && !isStopRequested()) ;

    }
}
