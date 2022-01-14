package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Frenzy", group = "Robot15173")
public class FrenzyModeRed extends FrenzyModeBase{
    protected void depositToTeamHub() {
        if (isButtonPressable()) {
            if (gamepad2.x) {
                startGamepadLockout();
                robot.dropToTeamHubRed();
            }
        }
    }

    protected void depositToSharedHub() {
        if (isButtonPressable()) {
            if (gamepad2.y) {
                startGamepadLockout();
                robot.dropToSharedHubRed();
            }
        }
    }
}
