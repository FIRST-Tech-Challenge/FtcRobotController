package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends OpMode {
    ViperSlide slide = null;

    @Override
    public void init() {
        slide = new ViperSlide(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            slide.up();
        } else if (gamepad1.a) {
            slide.down();
        } else {
            slide.stop();
        }
//        telemetry.add("Slide target: ", slide.getTarget());
    }
}
