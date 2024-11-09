package dev.aether.collaborative_multitasking

abstract class Scheduler {
    abstract fun task(configure: Task.() -> Unit): Task

    open fun isResourceInUse(resource: SharedResource): Boolean = false
    abstract fun tick()
    abstract fun getTicks(): Int
    internal abstract fun register(task: Task): Int
    abstract val nextId: Int

    abstract fun panic()

    abstract fun filteredStop(predicate: (Task) -> Boolean, cancel: Boolean)
    abstract fun filteredStop(predicate: (Task) -> Boolean)
    abstract fun filteredStop(
        predicate: (Task) -> Boolean,
        cancel: Boolean,
        dropNonStarted: Boolean
    )
}