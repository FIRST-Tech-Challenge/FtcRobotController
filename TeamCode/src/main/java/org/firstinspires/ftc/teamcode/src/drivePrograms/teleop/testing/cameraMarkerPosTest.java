package org.firstinspires.ftc.teamcode.src.drivePrograms.teleop.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutoObjDetectionTemplate;

import java.util.List;

@Disabled
@Autonomous(name = "Marker Position Test")

public class cameraMarkerPosTest extends AutoObjDetectionTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        this.initVuforia();
        this.initTfod();
        this.activateTF();
        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

        String recognition;
        double right;
        double left;
        double position;
        while (!isStarted() && !isStopRequested()) {
            if ((updatedRecognitions != null) && (updatedRecognitions.size() > 0)) {
                recognition = updatedRecognitions.get(0).getLabel();
                right = updatedRecognitions.get(0).getRight();
                left = updatedRecognitions.get(0).getLeft();
                position = (updatedRecognitions.get(0).getLeft() + updatedRecognitions.get(0).getRight()) / 2;

                telemetry.addData("recognition", recognition);
                telemetry.addData("left", left);
                telemetry.addData("right", right);
                telemetry.addData("position:", position);
                telemetry.update();
            }
        }

        waitForStart();


    }
}
