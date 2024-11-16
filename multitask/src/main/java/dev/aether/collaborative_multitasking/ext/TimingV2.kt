package dev.aether.collaborative_multitasking.ext

import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskTemplate

class Pause(scheduler: Scheduler, val seconds: Double) : TaskTemplate(scheduler) {
    private var startTime: Long = 0
    override fun invokeOnStart() {
        startTime = System.nanoTime()
    }

    override fun invokeIsCompleted(): Boolean {
        val now = System.nanoTime()
        return (now - startTime) / 1e9 >= seconds
    }
}