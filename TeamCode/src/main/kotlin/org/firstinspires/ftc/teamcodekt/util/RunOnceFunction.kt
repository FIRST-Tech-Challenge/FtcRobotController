package org.firstinspires.ftc.teamcodekt.util

fun <T> runOnce(block: () -> T): () -> T {
    var value: T? = null

    return {
        if (value == null) {
            value = block()
        }
        value!!
    }
}
