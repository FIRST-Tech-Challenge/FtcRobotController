package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Auto-aim that only targets BLUE AprilTag clusters (Limelight pipeline 3). */
@TeleOp(name = "Main TeleOp Blue", group = "Main")
public class MainTeleOpBlue extends MainTeleOpBase {

    @Override
    protected int getAlliance() {
        return ALLIANCE_BLUE;
    }
}