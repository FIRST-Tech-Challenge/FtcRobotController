package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCarousel extends HeadlessToggleableTool<DcMotor> {
    public static String carouselName = "spinner";

    public AutoCarousel(@NonNull HardwareMap hardwareMap, double power) {
        super(hardwareMap, DcMotor.class, carouselName, power * CarouselSpeedConfig.speed);
    }

    public AutoCarousel(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, 1);
    }

    @Config
    public static class CarouselSpeedConfig {
        public static double speed = 0.6;
    }
}
