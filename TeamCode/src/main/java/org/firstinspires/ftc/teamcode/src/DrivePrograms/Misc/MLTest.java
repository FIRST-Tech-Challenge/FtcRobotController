package org.firstinspires.ftc.teamcode.src.DrivePrograms.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.Utills.AutoObjDetectionTemplate;

@Disabled
@Autonomous(name = "MLTest")
public class MLTest extends AutoObjDetectionTemplate {
    @Override
    public void runOpMode() throws InterruptedException {

        this.initAll();
        this.initVuforia();
        this.initTfod();

        this.activate();
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
        while (opModeIsActive() && !isStopRequested()) ;

    }
}
