package org.firstinspires.ftc.teamcode.opMode.templates;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.RobotController;

import java.util.Arrays;
import java.util.HashSet;

public abstract class OpModeTemplate {
    private HashSet<Class<? extends Subsystem>> subsytems = new HashSet<>();
    @SafeVarargs
    protected OpModeTemplate(Class<? extends Subsystem>... subSystems) {
        this.subsytems.addAll(Arrays.asList(subSystems));
    }

    public HashSet<Class<? extends Subsystem>> getSubsystems() {
        return subsytems;
    };

    public void createControls(GamepadEx driverController, GamepadEx actionController, RobotController robotController) { };
}
