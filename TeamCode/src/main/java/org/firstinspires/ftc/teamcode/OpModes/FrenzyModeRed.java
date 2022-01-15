package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Red Frenzy", group = "Robot15173")
public class FrenzyModeRed extends FrenzyModeBase{

    @Override
    protected void depositToTeamHub() {
        if (isButtonPressable()) {
            if (gamepad2.x && !robotMoving) {

                startGamepadLockout();
                robot.dropToTeamHubRed();
            }
        }
    }

    @Override
    protected void depositToSharedHub() {
        if (isButtonPressable()) {
            if (gamepad2.y && !robotMoving) {
                startGamepadLockout();
                robot.dropToSharedHubRed();
            }
        }
    }

    @Override
    protected void handleTurntable() {
        if (isButtonPressable()) {
            if (gamepad2.b) {
                startGamepadLockout();
                robot.startTurntableRedGradual();
            }
        }
    }
}
