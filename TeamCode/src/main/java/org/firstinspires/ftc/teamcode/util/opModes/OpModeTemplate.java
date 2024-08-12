package org.firstinspires.ftc.teamcode.util.opModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

import java.util.Arrays;
import java.util.HashSet;

public abstract class OpModeTemplate {
    private final HashSet<Class<? extends SympleSubsystem>> subsystems = new HashSet<>();

    /**
     * @param subsystems the subsystem that the op mode template will have
     */
    @SafeVarargs
    protected OpModeTemplate(Class<? extends SympleSubsystem>... subsystems) {
        this.subsystems.addAll(Arrays.asList(subsystems));
    }

    public HashSet<Class<? extends SympleSubsystem>> getSubsystems() {
        return subsystems;
    }

    // runs after the subsystems are registered
    public void init(RobotController robotController) { }

    // runs after the subsystems are registered
    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) { }
}
