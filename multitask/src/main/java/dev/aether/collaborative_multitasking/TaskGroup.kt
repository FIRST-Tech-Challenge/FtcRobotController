package dev.aether.collaborative_multitasking

import java.util.function.Consumer

internal class TaskGroupScheduler : MultitaskScheduler() {
    val subtaskRequirements: MutableSet<SharedResource> = mutableSetOf()

    override fun register(task: ITask): Int {
        subtaskRequirements.addAll(task.requirements())
        return super.register(task)
    }
}

class TaskGroup(private val outerScheduler: Scheduler) : TaskTemplate(outerScheduler) {
    private val innerScheduler = TaskGroupScheduler()

    fun with(provider: (Scheduler) -> Unit): TaskGroup {
        provider(outerScheduler)
        return this
    }

    fun with(provider: Consumer<Scheduler>): TaskGroup {
        provider.accept(outerScheduler)
        return this
    }

    override fun requirements(): Set<SharedResource> {
        return innerScheduler.subtaskRequirements
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

    private val genName: String; get() = "with ${innerScheduler.taskCount()} inner tasks"
    private var customName = "unnamed"
    override var name: String
        get() = "$customName, $genName"
        set(value) {
            customName = value
        }

    override fun display(indent: Int, write: (String) -> Unit) {
        innerScheduler.displayStatus(false, false, write, indent)
    }
}