package org.firstinspires.ftc.teamcode.util.opModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubsystem;

import java.util.Arrays;
import java.util.HashSet;

public abstract class OpModeTemplate {
    private final HashSet<Class<? extends SympleSubsystem>> subsytems = new HashSet<>();
    @SafeVarargs
    protected OpModeTemplate(Class<? extends SympleSubsystem>... subSystems) {
        this.subsytems.addAll(Arrays.asList(subSystems));
    }

    public HashSet<Class<? extends SympleSubsystem>> getSubsystems() {
        return subsytems;
    }

    public void init(RobotController robotController) { }

    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) { }
}
