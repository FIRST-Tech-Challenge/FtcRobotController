package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Auto-aim that only targets RED AprilTag clusters (Limelight pipeline 3). */
@TeleOp(name = "Main TeleOp Red", group = "Main")
public class MainTeleOpRed extends MainTeleOpBase {

    @Override
    protected int getAlliance() {
        return ALLIANCE_RED;
    }
}