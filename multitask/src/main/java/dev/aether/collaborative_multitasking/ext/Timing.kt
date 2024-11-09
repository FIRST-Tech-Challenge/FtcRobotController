@file:Suppress("unused")

package dev.aether.collaborative_multitasking.ext

import dev.aether.collaborative_multitasking.Task

/**
 * Delay task (destructive)
 */
fun Task.delay(millis: Int) {
    var startTime = System.currentTimeMillis()
    onStart { ->
        startTime = System.currentTimeMillis()
    }
    isCompleted { ->
        System.currentTimeMillis() - startTime >= millis
    }
}

/**
 * The task cannot complete until the given number of milliseconds have passed.
 * DO NOT USE AS A DELAY WITHOUT OVERRIDING isCompleted!!!
 */
fun Task.minDuration(millis: Int) {
    var startTime = System.currentTimeMillis()
    extendOnStart { ->
        startTime = System.currentTimeMillis()
    }
    isCompletedAnd { ->
        System.currentTimeMillis() - startTime >= millis
    }
}

fun Task.minTicks(ticks: Int) {
    isCompletedAnd { _, scheduler ->
        scheduler.getTicks() - this.startedAt!! >= ticks
    }
}

/**
 * The task will forcibly complete after the given number of milliseconds have passed.
 */
fun Task.maxDuration(millis: Int) {
    var startTime = System.currentTimeMillis()
    extendOnStart { ->
        startTime = System.currentTimeMillis()
    }
    isCompletedOr { ->
        System.currentTimeMillis() - startTime >= millis
    }
}
