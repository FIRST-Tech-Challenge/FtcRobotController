package dev.aether.collaborative_multitasking

import java.util.function.Consumer

internal class TaskGroupScheduler : MultitaskScheduler() {
    val subtaskRequirements: MutableSet<SharedResource> = mutableSetOf()

    override fun register(task: ITask): Int {
        subtaskRequirements.addAll(task.requirements())
        return super.register(task)
    }
}

class TaskGroup(outerScheduler: Scheduler) : TaskTemplate(outerScheduler) {
    private val innerScheduler = TaskGroupScheduler()

    /**
     * Access the inner scheduler, cast to a generic Scheduler. Not recommended for general use.
     */
    fun accessInner() = innerScheduler as Scheduler

    fun with(provider: (Scheduler) -> Unit): TaskGroup {
        provider(innerScheduler)
        return this
    }

    fun with(provider: Consumer<Scheduler>): TaskGroup {
        provider.accept(innerScheduler)
        return this
    }

    private val extraDeps: MutableSet<SharedResource> = mutableSetOf()

    fun extraDepends(vararg with: SharedResource): TaskGroup {
        extraDeps.addAll(with)
        return this
    }

    fun extraClear() = extraDeps.clear()

    override fun requirements(): Set<SharedResource> {
        return innerScheduler.subtaskRequirements + extraDeps
    }

    override fun invokeOnTick() {
        innerScheduler.tick()
    }

    override fun invokeOnFinish() {
        // run onFinish handlers for inner tasks
        innerScheduler.filteredStop { true }
    }

    override fun invokeIsCompleted(): Boolean {
        return !innerScheduler.hasJobs()
    }

    private val genName: String; get() = "(a group containing ${innerScheduler.taskCount()} subtasks)"
    private var customName = "unnamed"
    override var name: String
        get() = "$customName $genName"
        set(value) {
            customName = value
        }

    override fun display(indent: Int, write: (String) -> Unit) {
        innerScheduler.displayStatus(false, false, write, indent)
    }
}