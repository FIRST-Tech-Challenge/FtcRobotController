package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Blue Frenzy", group = "Robot15173")
public class FrenzyModeBlue extends FrenzyModeBase {
    @Override
    protected void depositToTeamHub() {
    if (isButtonPressable()) {
        if (gamepad2.x) {
            startGamepadLockout();
            robot.extendToTeamHubBlueAsync();
        }
    }
}
    @Override
    protected void depositToSharedHub() {
        if (isButtonPressable()) {
            if (gamepad2.y) {
                startGamepadLockout();
                robot.extendToSharedHubBlueAsync();
            }
        }
    }

    @Override
    protected void handleTurntable() {
        if (isButtonPressable()) {
            if (gamepad1.b) {
                startGamepadLockout();
                robot.startTurntableBlueGradual();
            }
        }
    }
}

