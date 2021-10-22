package org.firstinspires.ftc.teamcode.parts;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;
import androidx.annotation.NonNull;

public class ControllerMovement {
    private final GamepadEx gamepad;
    private final StrafingMovement move;
    public void update() {
        move.driveDRS(gamepad.getLeftY(), 0, gamepad.getLeftX());
    }
    public ControllerMovement(@NonNull HardwareMap map, GamepadEx gamepad) {
        this.move = new StrafedMovementImpl(map);
        this.gamepad = gamepad;
    }
}
