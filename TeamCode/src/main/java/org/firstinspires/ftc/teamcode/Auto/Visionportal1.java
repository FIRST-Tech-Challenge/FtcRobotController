// "builder," not "easy"

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(group="Concept")
public class Visionportal1 extends LinearOpMode {

    private void initTfod() {
        TfodProcessor myTfodProcessor;
        myTfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10) // max # recognitions
                .setUseObjectTracker(true) // use object tracker
                .setTrackerMaxOverlap((float) 0.2) // max % of box overlapped by another box for recognition
                .setTrackerMinSize(16) // minimum size of a tracked/recognized object (units?)
                .build();
    }


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        if (!isStopRequested()) {
            while(opModeIsActive()) {

                telemetry.update();
            }
        }
    }

}
