package org.firstinspires.ftc.teamcode.internals.input

class TwoWayToggleInput(
) {
    private var pressed: Boolean = false
    private var state: Double = 0.0
    fun input(
        input: () -> Boolean,
        idle: Double = 0.0,
        power: Double = 1.0,
    ){
        if(input() && !pressed){
            state = power
            pressed = true
        }
        if(input() && pressed){
            state = idle
            pressed = false
        }
    }
}