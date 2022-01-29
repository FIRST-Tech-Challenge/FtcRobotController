package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Frenzy", group = "Robot15173")
public class FrenzyModeRed extends FrenzyModeBase{

    @Override
    protected void depositToTeamHub() {
        if (isGamepad1Pressable()) {
            if (gamepad2.x) {
                lockGamepad2();
                robot.extendToTeamHubRedAsync();
            }
        }
    }

    @Override
    protected void depositToSharedHub() {
        if (isGamepad1Pressable()) {
            if (gamepad2.y) {
                lockGamepad2();
                robot.extendToSharedHubRedAsync();
            }
        }
    }

    @Override
    protected void handleTurntable() {
        if (isGamepad1Pressable()) {
            if (gamepad1.b) {
                lockGamepad1();
                robot.startTurntableRedGradual();
            }
        }
    }
}
