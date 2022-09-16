package org.firstinspires.ftc.teamcode.stateMachine

import robotuprising.lib.system.statemachine.transition.TransitionCondition

data class State<StateEnum>(
    var state: StateEnum,
    var enterActions: MutableList<Action>,
    var exitActions: MutableList<Action>,
    var loopActions: MutableList<Action>,
    var transitionCondition: TransitionCondition?
)
