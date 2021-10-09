package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Point;

public class RobotClassInnerBlueOuterRed extends RobotClass {

    public RobotClassInnerBlueOuterRed(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode, String color) {
        super(hardwareMap, telemetry, opmode, color);
        REGION1_TOPLEFT_ANCHOR_POINT= new Point(68,176);
    }
}