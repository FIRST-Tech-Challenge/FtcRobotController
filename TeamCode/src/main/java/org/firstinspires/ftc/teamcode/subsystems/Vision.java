package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class Vision extends SubsystemBase {
    Telemetry m_telemetry;
    Telemetry.Item tItem;
    VuforiaLocalizer m_vuforia;
    HardwareMap m_hardwareMap;
    TFObjectDetector m_tfod;

    static final String VUFORIA_KEY = "ARKNcpL/////AAABmaul75WJu02hpEsBG/MnvsZ0aacsUMH0zc+d53A" +
            "KGDU3mzdXJQzSDPuea0rokovM0/U3INJNoaNvGx+Xnk9tFdgMVitg+hE32fMsH4f5KLF9CqJyqRynTBo55jfOsN4UbPMO6ij" +
            "MdpQg7PPUg8O7pd5HNOjoDx+MKgZ+FldA4uCCj5vEansKoP5++7V0a/E/0j4MClpdkUA/7cf9WSNS8opUnl9lAyNpOEiOa0b" +
            "q2KbzC5234XlaqzE7it5yl9QhstUyAfy1rRyRYc7ClclkuK1kleXepW2FQED5MsC3S+4buqtAe2pnJA7QyHJ3PGUBQd3L5PF" +
            "VVDeXRGHIF6ZKij3R6zKbWc6/NVSc2J7S5Uz2";

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        m_telemetry = telemetry;
        m_hardwareMap = hardwareMap;

        initVuforia();
        initTfod();

        if (m_tfod != null) {
            m_tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            m_tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    @Override
    public void periodic() {
        if (m_tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = m_tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                tItem = m_telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                    tItem.addData(String.format("label (%d)", i), recognition.getLabel());
                    tItem.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                            recognition.getLeft(), recognition.getTop());
                    tItem.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                            recognition.getRight(), recognition.getBottom());
                    i++;
                }
            }
        }
        tItem.setRetained(true);
        m_telemetry.update();
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = m_hardwareMap.get(WebcamName.class, "camera");

        //  Instantiate the Vuforia engine
        m_vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = m_hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", m_hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        m_tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, m_vuforia);
        m_tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

}
