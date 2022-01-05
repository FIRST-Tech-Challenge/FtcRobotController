package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class FreightFrenzyComputerVisionBlueHub extends FreightFrenzyComputerVisionRedHub {

    public FreightFrenzyComputerVisionBlueHub(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, telemetry, "blue");

    }

}
