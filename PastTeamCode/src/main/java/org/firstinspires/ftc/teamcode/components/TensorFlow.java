package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.TfodCurrentGame;
import org.firstinspires.ftc.teamcode.helpers.ObjectEnums;

import java.util.EnumMap;
import java.util.List;

public class TensorFlow {

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    private VuforiaCurrentGame vuforiaFreightFrenzy;
    private TfodCurrentGame tfodFreightFrenzy;

    private static final String VUFORIA_KEY = "Ad0Srbr/////AAABmdpa0/j2K0DPhXQjE2Hyum9QUQXZO8uAVCNpwlogfxiVmEaSuqHoTMWcV9nLlQpEnh5bwTlQG+T35Vir8IpdrSdk7TctIqH3QBuJFdHsx5hlcn74xa7AiQSJgUD/n7JJ2zJ/Er5Hc+b+r616Jf1YU6RO63Ajk5+TFB9N3a85NjMD6eDm+C6f14647ELnmGC03poSOeczbX7hZpIEObtYdVyKZ2NQ/26xDfSwwJuyMgUHwWY6nl6mk0GMnIGvu0/HoGNgyR5EkUQWyx9XlmxSrldY7BIEVkiKmracvD7W9hEGZ2nPied6DTY5RFNuFX07io6+I59/d7291NXKVMDnFAqSt4a2JYsECv+j7b25S0mD";

    private HardwareMap hardwareMap;



    /**
     * Constructor for TensorFlow
     */
    public TensorFlow(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /** Method getInference()
     *
     * Returns a list of Tensoflow recognitions
     *
     * A recognition is, from my understanding, TensorFlow having identifying
     * an object as one of its labels - the thing it should be looking for.
     *
     * In a sense, this is sort of a 'picture' of all of the relevant
     * details or aspects of the picture. This is good so that we can
     * iterate through these Recognitions and find important information,
     * such as which label (or type) - amongst other things - these Recognitions are.
     *
     * Your garbage explanation brought to you by: @anishch
     */

    public List<Recognition> getInference() { //get "image" back, a bunch of Recognitions - check out instance variables
        if (tfodFreightFrenzy != null) {
            return tfodFreightFrenzy.getRecognitions(); //Returns the list of recognitions, but only if they are different than the last call to {@link #getUpdatedRecognitions()}.
        }
        return null;
    }

    /**
     * Activates TensorFlow
     */
    public void activate() {


        if (tfodFreightFrenzy == null) {
            List<Recognition> recognitions;
            int index;

            vuforiaFreightFrenzy = new VuforiaCurrentGame();
            tfodFreightFrenzy = new TfodCurrentGame();

            // Sample TFOD Op Mode
            // Initialize Vuforia.
            // This sample assumes phone is in landscape mode.
            // Rotate phone -90 so back camera faces "forward" direction on robot.
            vuforiaFreightFrenzy.initialize(
                    VUFORIA_KEY, // vuforiaLicenseKey
                    hardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                    "", // webcamCalibrationFilename
                    false, // useExtendedTracking
                    true, // enableCameraMonitoring
                    VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                    0, // dx
                    0, // dy
                    0, // dz
                    AxesOrder.XZY, // axesOrder
                    90, // firstAngle
                    90, // secondAngle
                    0, // thirdAngle
                    true); // useCompetitionFieldTargetLocations
            vuforiaFreightFrenzy.activate();
            // Set min confidence threshold to 0.7
            tfodFreightFrenzy.initialize(vuforiaFreightFrenzy, (float) 0.7, true, true);
            // Initialize TFOD before waitForStart.
            // Init TFOD here so the object detection labels are visible
            // in the Camera Stream preview window on the Driver Station.
            tfodFreightFrenzy.activate();
            // Enable following block to zoom in on target.
            tfodFreightFrenzy.setZoom(4, 16 / 9);
        }


    }

    /**
     * Shuts down TensorFlow
     */
    public void shutdown() {
        if (tfodFreightFrenzy != null) {
            tfodFreightFrenzy.deactivate();
            vuforiaFreightFrenzy.deactivate();
        }
    }

    public boolean seesDuck(){
        List<Recognition> recognitions = getInference();
        if(recognitions.size() > 0) {
            for (int i = 0; i < recognitions.size(); i++) {
                String label = recognitions.get(i).getLabel();
                if (label.equals(LABELS[1]) || label.equals(LABELS[2])) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Returns the target region currently selected
     * @return the target region currently selected
     */
    /*public ObjectEnums getObject() {
        if (tfod == null) {
            return ObjectEnums.NADA;
        }
        List<Recognition> recognitionList = getInference();
        if (recognitionList.size() == 1 && recognitionList.get(0).getConfidence() >= 0.4) {
            if (recognitionList.get(0).getLabel().equals(LABELS[0])){
                return ObjectEnums.BALL;
            }
            if (recognitionList.get(0).getLabel().equals(LABELS[1])){
                return ObjectEnums.CUBE;
            }
            if (recognitionList.get(0).getLabel().equals(LABELS[2])){
                return ObjectEnums.DUCK;
            }
            if (recognitionList.get(0).getLabel().equals(LABELS[3])){
                return ObjectEnums.MARKER;
            }
        }
        return ObjectEnums.NADA;
    }*/
}