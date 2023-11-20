package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="drive")
public class thingy extends LinearOpMode {
    HuskyLens husky;
    @Override
    public void runOpMode() throws InterruptedException {
        husky = hardwareMap.get(HuskyLens.class, "huskyDog");
        HuskyLens.Block[] blocks = husky.blocks();
        HuskyLens.Arrow[] arrows = husky.arrows();
        telemetry.addLine("blocks " + blocks.toString() + ", arrows " + arrows.toString());
    }
}
