package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

public class ViperSlide {
    private DcMotor slide = null;

    private int target = 0;

    // Define the floor and ceiling of ViperSlide movement
    final int ceil = 300;
    final int floor = 0;

    public ViperSlide(HardwareMap hardwareMap) {
        slide = hardwareMap.get(DcMotor.class, "Slide");
        slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getTarget() {
        return target;
    }

    public void up() {
        target += 10;
        if (target > ceil) {
            target = ceil;
        }
        slide.setTargetPosition(target);
    }

    public void down() {
        target -= 10;
        if (target < floor) {
            target = floor;
        }
        slide.setTargetPosition(target);
    }

    public void stop() {
        slide.setPower(0);
    }


}
