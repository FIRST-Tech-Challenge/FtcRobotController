package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class RBDetector {
    RBPipeline pipeline;

    public RBDetector(HardwareMap hardwareMap) {
        pipeline = new RBPipeline();
        pipeline.setupWebcam(hardwareMap);
    }

    public String debug() {
        return pipeline.position.toString() + " " + pipeline.debug;
    }

    void dispose() {
        pipeline.disposeWebcam();
    }
}
