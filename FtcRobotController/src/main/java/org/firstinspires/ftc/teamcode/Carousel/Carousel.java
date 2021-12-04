package org.firstinspires.ftc.teamcode.Carousel;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    private DcMotor carouselTurningMotor;
    private DigitalChannel touchSensor;

    // Constructor
    public Carousel(HardwareMap hardwareMap) {
        carouselTurningMotor = hardwareMap.dcMotor.get("carouselTurningMotor");
        touchSensor = hardwareMap.digitalChannel.get("carouselTouchSensor");

        // Sets the touch sensor mode to an input so we can call getState()
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public void CarouselHandler() {
        // if the state of touch sensor is true, turn on motor. if not then set power to 0
        if (touchSensor.getState()) {
            carouselTurningMotor.setPower(0.2);
        } else {
            carouselTurningMotor.setPower(0);
        }
    }
}
