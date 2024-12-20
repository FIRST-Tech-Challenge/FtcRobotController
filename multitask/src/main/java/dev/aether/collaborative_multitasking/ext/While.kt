package dev.aether.collaborative_multitasking.ext

import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskTemplate

class While(
    scheduler: Scheduler,
    val condition: () -> Boolean,
    val action: () -> Unit
) : TaskTemplate(scheduler) {
    constructor(scheduler: Scheduler, condition: () -> Boolean, action: Runnable) : this(
        scheduler,
        condition,
        action::run
    )

    override fun invokeIsCompleted(): Boolean = !(condition())

    override fun invokeOnTick() = action()
}