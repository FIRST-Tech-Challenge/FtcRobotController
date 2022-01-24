package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Frenzy", group = "Robot15173")
public class FrenzyModeBlue extends FrenzyModeBase {
    @Override
    protected void depositToTeamHub() {
    if (isButtonPressable()) {
        if (gamepad2.x) {
            startGamepadLockout();
            robot.extendToTeamHubBlue();
        }
    }
}
    @Override
    protected void depositToSharedHub() {
        if (isButtonPressable()) {
            if (gamepad2.y) {
                startGamepadLockout();
                robot.dropToSharedHubBlue();
            }
        }
    }

    @Override
    protected void handleTurntable() {
        if (isButtonPressable()) {
            if (gamepad2.b) {
                startGamepadLockout();
                robot.startTurntableBlueGradual();
            }
        }
    }
}

