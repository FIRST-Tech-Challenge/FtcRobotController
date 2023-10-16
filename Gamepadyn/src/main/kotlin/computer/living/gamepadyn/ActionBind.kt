package computer.living.gamepadyn

import computer.living.gamepadyn.ActionType.ANALOG
import computer.living.gamepadyn.ActionType.DIGITAL

sealed interface ActionBind {
    abstract val type: ActionType
}

class AnalogActionBind() : ActionBind {
    override val type: ActionType = ANALOG
}

class DigitalActionBind() : ActionBind {
    override val type: ActionType = DIGITAL

}