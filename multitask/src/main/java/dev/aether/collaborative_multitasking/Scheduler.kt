package dev.aether.collaborative_multitasking

abstract class Scheduler {
    abstract fun task(configure: Task.() -> Unit): Task
    abstract fun task(t: ITask): ITask

    open fun isResourceInUse(resource: SharedResource): Boolean = false
    abstract fun tick()
    abstract fun getTicks(): Int
    internal abstract fun register(task: ITask): Int
    abstract val nextId: Int

    abstract fun panic()

    abstract fun filteredStop(predicate: (ITask) -> Boolean, cancel: Boolean)
    abstract fun filteredStop(predicate: (ITask) -> Boolean)
    abstract fun filteredStop(
        predicate: (ITask) -> Boolean,
        cancel: Boolean,
        dropNonStarted: Boolean
    )

    abstract fun taskCount(): Int
}