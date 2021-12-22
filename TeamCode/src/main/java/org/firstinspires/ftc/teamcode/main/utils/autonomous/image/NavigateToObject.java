package org.firstinspires.ftc.teamcode.main.utils.autonomous.image;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.ArrayList;
import java.util.List;

@TeleOp(group = "Concept")
public class NavigateToObject extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BC.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube"
    };

    private String VUFORIA_KEY;

    private double lowestY;
    private int lowestYPos;

    private TFLITE_Wrapper tflite;

    private ArrayList<InitialPositions> PossiblePositions;

    private boolean isZoomed = false;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        tflite = new TFLITE_Wrapper(hardwareMap);

        tflite.confidence = 0.6f;

        tflite.init();

        if (tflite.tfod != null) {
            tflite.activate();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (tflite.tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                lowestY=0;
                List<Recognition> updatedRecognitions = tflite.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        telemetry.addData(String.format("  angle (%d)", i),
                                String.format("%.03f", recognition.estimateAngleToObject(AngleUnit.DEGREES)));

                        if (recognition.getTop() > lowestY) {
                            lowestY = recognition.getTop();
                            lowestYPos = i;
                        }
                        i++;
                    }

                    telemetry.addData("Closest Object", lowestYPos);
                    telemetry.addData("Closest Object Position", lowestY);


                    if (gamepad1.circle) {
                        if (isZoomed) {
                            tflite.setZoom(1.0, 16.0 / 9.0);
                        } else {
                            tflite.setZoom(1.5, 16.0 / 9.0);
                        }

                        isZoomed = !isZoomed;

                        sleep(100);
                    }

                    telemetry.update();

                }
            }
        }
    }
}
