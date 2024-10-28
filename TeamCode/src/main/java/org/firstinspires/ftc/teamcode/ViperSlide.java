package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class ViperSlide {
    private DcMotor slide = null;

    public ViperSlide(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotor.class, "Slide");
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void up() {
        slide.setTargetPosition(200);
    }

    public void down() {
        slide.setTargetPosition(0);
    }

    public void stop() {
        slide.setPower(0);
    }
}
