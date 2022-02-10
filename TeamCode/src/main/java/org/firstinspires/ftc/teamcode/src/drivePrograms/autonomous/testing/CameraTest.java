package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

import java.util.List;
import java.util.Locale;

@Disabled
@TeleOp(name = "HI Son of Carl")
public class CameraTest extends AutoObjDetectionTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        initAll();
        while (!isStopRequested()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format(Locale.ENGLISH, "label (%d)", i), recognition.getLabel());
                        double cx = (recognition.getLeft() + recognition.getRight()) / 2.0;
                        double cy = (recognition.getTop() + recognition.getBottom()) / 2.0;
                        telemetry.addData("Cx", cx);
                        telemetry.addData("Cy", cy);
                        telemetry.update();

                        i++;
                    }
                    telemetry.update();
                }
            }
        }


        waitForStart();


    }
}
