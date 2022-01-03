package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoCarousel extends HeadlessToggleableTool<CRServo> {
    public AutoCarousel(@NonNull HardwareMap hardwareMap, double power) {
        super(hardwareMap, CRServo.class, "spinner", power);
    }

    public AutoCarousel(@NonNull HardwareMap hardwareMap) {
        this(hardwareMap, 1);
    }
}
