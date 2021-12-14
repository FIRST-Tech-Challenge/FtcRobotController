package org.firstinspires.ftc.teamcode.core.robot;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;
import androidx.annotation.NonNull;

public class ControllerMovement {
    private final GamepadEx gamepad;
    private final StrafingMovement move;

    public ControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        this.move = new StrafedMovementImpl(map);
        this.gamepad = gamepad;
    }

    public void update() {
        if (gamepad.getButton(GamepadKeys.Button.X)) {
            move.driveDRS(0, 1, 0);

        } else {
            move.driveDRS(gamepad.getLeftY(), 0, gamepad.getLeftX());
        }
    }

    public double[] motorVelocities() {
        return ((StrafedMovementImpl) move).motorVelocities();
    }
}
