package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot2022 extends Robot {
    private Intake intake;
    private Carousel carousel;
    private Bucket bucket;
    private Slider slider;

    public Robot2022(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1,
                     Gamepad gamepad2) {
        super(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void initMechanisms(HardwareMap hardwareMap) {
        intake = new Intake(hardwareMap, gamepad2);
        slider = new Slider(hardwareMap, telemetry, gamepad2);
        carousel = new Carousel(hardwareMap, gamepad2);
        bucket = new Bucket(hardwareMap, gamepad2);
    }

    @Override
    public void runMechanisms() {
        intake.run();
        carousel.run();
        bucket.run();
        slider.run(intake, bucket);
    }
}