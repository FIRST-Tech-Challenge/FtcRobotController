package dev.aether.collaborative_multitasking

import java.lang.Runnable

class OneShot(scheduler: Scheduler, val target: () -> Unit) : TaskTemplate(scheduler) {
    constructor(scheduler: Scheduler, target: Runnable) : this(scheduler, target::run)

    override fun invokeOnStart() {
        target()
    }

    override fun invokeIsCompleted() = true
}