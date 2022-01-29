package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Frenzy", group = "Robot15173")
public class FrenzyModeBlue extends FrenzyModeBase {
    @Override
    protected void depositToTeamHub() {
    if (isGamepad2Pressable()) {
        if (gamepad2.x) {
            lockGamepad2();
            robot.extendToTeamHubBlueAsync();
        }
    }
}
    @Override
    protected void depositToSharedHub() {
        if (isGamepad2Pressable()) {
            if (gamepad2.y) {
                lockGamepad2();
                robot.extendToSharedHubBlueAsync();
            }
        }
    }

    @Override
    protected void handleTurntable() {
        if (isGamepad1Pressable()) {
            if (gamepad1.b) {
                lockGamepad1();
                robot.startTurntableBlueGradual();
            }
        }
    }
}

