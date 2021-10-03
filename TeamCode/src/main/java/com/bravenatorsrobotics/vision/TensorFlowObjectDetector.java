package com.bravenatorsrobotics.vision;

import com.bravenatorsrobotics.core.BravenatorRuntimeException;
import com.bravenatorsrobotics.operation.OperationMode;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class TensorFlowObjectDetector {

    // ============================================================
    // Constants
    // ============================================================

    // Filename for where the tfod information is stored.
    private static final String TFOD_MODEL_ASSET_FILENAME = "FreightFrenzy_BCDM.tflite"; // From ConceptTensorFlowObjectDetection.java
    private static final String[] OBJECT_LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private static final String VUFORIA_KEY =
            "AYMJwEL/////AAABmXhnZUthlkQRqZtBBpNME36DsqdlCz6tt/5wuog/yF9PmuuAhfZZ8uAPCgQJ6WXXjB+fDSCT9z0knxMTAhrZ0Ey/0zpe5MGvDnae7MKDlFxK/GiUlr5PFSpZ+afiAYK7fc0SQOEjf6Oup+gRzn9sDe+3sZzkgTNQ7iSQ+NIltPSEQEO1lpyC7+fG47ya5i9YSA5CxZW1+R9J3FYkqvh+gMSniRAjeXY97wl3dPmahCuAcTSx+WYSijLQX6PLpiHyEe+Q1qAGdimhfq/mjrE1iWQU9OMVNAIoC0lZ+RmJr3+lPg8gR0UhjRLp9v3M1Giv08siHwRdlvb24W49SRdLNSqPqCxAcBvr0l1BkrpIbtIh";

    // ============================================================
    // Member Variables
    // ============================================================

    private final OperationMode<?> opMode;
    private final String webcamName;

    private VuforiaLocalizer vuforia = null;
    private TFObjectDetector tfod = null;

    private List<Recognition> recognitions = null;

    public TensorFlowObjectDetector(OperationMode<?> opMode, String webcamName) {
        this.opMode = opMode;
        this.webcamName = webcamName;
    }

    public void Initialize() {
        // Vuforia
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, webcamName);

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // TFOD
        int tfodMonitorViewID = opMode.hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewID", "id", opMode.hardwareMap.appContext.getPackageName());

        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewID);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET_FILENAME, OBJECT_LABELS);

        if(tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
        } else {
            throw new BravenatorRuntimeException("Could not initialize TFOD!");
        }
    }

    public void UpdateRecognitions() {

        if(tfod != null) {
            System.out.println("Hey");

            recognitions = tfod.getUpdatedRecognitions();

            if(recognitions != null && recognitions.size() > 0) {

                for(Recognition recognition : recognitions) {
                    opMode.telemetry.addLine(opMode.telemetry.getItemSeparator());
                    opMode.telemetry.addLine(recognition.getLabel());
                    opMode.telemetry.addLine(String.format(Locale.US, "left = %.3f, right = %.3f", recognition.getLeft(), recognition.getRight()));
                    opMode.telemetry.addLine(String.format(Locale.US, "top = %.3f, bottom = %.3f", recognition.getTop(), recognition.getBottom()));

                    recognition.getLeft();
                }

                opMode.telemetry.update();
            }

        }
    }
}
