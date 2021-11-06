package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CarouselPseudocode {
    private DcMotor carouselTurningMotor;
    private DigitalChannel touchSensor;

    // Constructor
    public CarouselPseudocode(HardwareMap hardwareMap) {
        carouselTurningMotor = hardwareMap.dcMotor.get("motorCarousel");
        touchSensor = hardwareMap.digitalChannel.get("CarouselTouchSensor");
    }

    public void CarouselHandler() {


    }

}
