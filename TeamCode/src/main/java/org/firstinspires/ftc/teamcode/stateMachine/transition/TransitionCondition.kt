package org.firstinspires.ftc.teamcode.stateMachine.transition

fun interface TransitionCondition {
    fun shouldTransition(): Boolean
}
