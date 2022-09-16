package org.firstinspires.ftc.teamcode.stateMachine

fun interface TransitionCondition {
    fun shouldTransition(): Boolean
}