package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

import java.util.List;
import java.util.Locale;

/**
 * A Autonomous that tests our machine learning tools
 */
@Disabled
@Autonomous(name = "TeamModelObjectDetectionTest")
public class TeamModelObjectDetectionTest extends AutoObjDetectionTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        initAll();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
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
                            telemetry.addData(String.format(Locale.ENGLISH, "  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format(Locale.ENGLISH, "  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            i++;
                        }
                        telemetry.update();
                    }


                }
            }
        }
        waitForStart();

    }

}


