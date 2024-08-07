package org.firstinspires.ftc.teamcode.util.opModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.subsystems.SympleSubSystemBase;

import java.util.Arrays;
import java.util.HashSet;

public abstract class OpModeTemplate {
    private final HashSet<Class<? extends SympleSubSystemBase>> subsytems = new HashSet<>();
    @SafeVarargs
    protected OpModeTemplate(Class<? extends SympleSubSystemBase>... subSystems) {
        this.subsytems.addAll(Arrays.asList(subSystems));
    }

    public HashSet<Class<? extends SympleSubSystemBase>> getSubsystems() {
        return subsytems;
    }

    public void init(RobotController robotController) { }

    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) { }
}
