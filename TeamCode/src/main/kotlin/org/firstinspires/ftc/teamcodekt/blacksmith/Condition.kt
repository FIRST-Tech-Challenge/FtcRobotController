package org.firstinspires.ftc.teamcodekt.blacksmith

fun interface Condition {
    fun evaluate(): Boolean

    operator fun invoke(): Boolean {
        return evaluate()
    }
}
