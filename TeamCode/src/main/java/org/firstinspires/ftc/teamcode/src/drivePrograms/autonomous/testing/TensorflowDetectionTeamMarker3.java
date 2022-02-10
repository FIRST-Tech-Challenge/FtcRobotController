package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

import java.util.List;
import java.util.Locale;

@Disabled
@Autonomous(name = "New Detection Method Test")
public class TensorflowDetectionTeamMarker3 extends AutoObjDetectionTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        initAll();
        while (!isStopRequested()) {
            List<Recognition> recognitions = tfod.getRecognitions();
            if (recognitions == null || (recognitions.size() == 0)) {
                telemetry.addData("Object", "Empty list");
                telemetry.update();
                continue;
            } else {
                Recognition recognition = recognitions.get(0);
                double xCenter = (recognition.getRight() + recognition.getLeft()) / 2;
                double yCenter = (recognition.getTop() + recognition.getBottom()) / 2;
                telemetry.addData("Object", this.findPositionOfMarker());
                telemetry.addData("Location", String.format(Locale.ENGLISH, "(%f,%f)", xCenter, yCenter));
            }

            telemetry.update();
        }

    }
}
