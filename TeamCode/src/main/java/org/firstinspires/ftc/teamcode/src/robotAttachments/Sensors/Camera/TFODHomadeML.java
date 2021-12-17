package org.firstinspires.ftc.teamcode.src.robotAttachments.Sensors.Camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@Deprecated
public class TFODHomadeML extends TFOD {

    // This class will access the file and label of our homemade machine learning tool's model
    protected static final String TFOD_MODEL_ASSET = "";
    protected static final String[] LABELS = {""};

    public TFODHomadeML(Parameters tfodParameters, VuforiaLocalizer vuforiaLocalizer, HardwareMap hardwareMap) {
        super(tfodParameters, vuforiaLocalizer, hardwareMap);
    }


}
