package org.firstinspires.ftc.teamcode.components.easytoggle

class EasyToggle (initialState: Boolean = false) {
    var state = initialState
        set(value) {
            previousState = field
            field = value
        }

    private var previousState = initialState

    fun nowTrue() = state && !previousState
    fun nowFalse() = !state && previousState
}