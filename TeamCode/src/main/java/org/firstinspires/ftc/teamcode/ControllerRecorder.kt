package org.firstinspires.ftc.teamcode

import org.firstinspires.ftc.teamcode.util.GamepadState
import java.util.ArrayList

class ControllerRecorder {
    private var gamepadStates: ArrayList<GamepadState> = ArrayList<GamepadState>()

    fun addState(state: GamepadState) {
        gamepadStates.add(state)
    }

    // I'm going mad I can't save files
    fun save() {

    }
}

// I like it thick and fried