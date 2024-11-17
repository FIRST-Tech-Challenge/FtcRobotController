package dev.aether.collaborative_multitasking

import dev.aether.collaborative_multitasking.ITask.State

interface ITask {
    enum class State(val order: Int) {
        NotStarted(0),
        Starting(1),
        Ticking(2),
        Finishing(3),
        Finished(4),
        Cancelled(4),
    }

    val scheduler: Scheduler
    val state: State
    val myId: Int?
    var name: String
    val daemon: Boolean
    val isStartRequested: Boolean
    fun onRequest() = isStartRequested

    // Lifecycle
    fun transition(newState: State)
    fun register()

    fun invokeCanStart(): Boolean
    fun invokeOnStart()
    fun invokeOnTick()
    fun invokeIsCompleted(): Boolean
    fun invokeOnFinish()

    fun requirements(): Set<SharedResource>
    infix fun waitsFor(after: ITask)

    fun requestStart()
    fun requestStop(cancel: Boolean) {
        scheduler.filteredStop({ it == this }, cancel)
    }

    fun requestStop() = requestStop(true)

    fun then(configure: Task.() -> Unit): Task {
        val task = Task(scheduler)
        task.name = MultitaskScheduler.getCaller()
        task.configure()
        task waitsFor this
        task.register() // ready to go
        return task
    }

    fun then(task: ITask): ITask {
        task waitsFor this
        task.register()
        return task
    }

    fun then(polyChain: Pair<Task, Task>): Task {
        this.then(polyChain.first)
        return polyChain.second
    }
}

abstract class TaskWithWaitFor() : ITask {

    private var waitFor: MutableSet<ITask> = mutableSetOf()

    fun getWaitsFor() = waitFor as Set<ITask>

    override fun waitsFor(after: ITask) {
        waitFor.add(after)
    }

    override fun invokeCanStart(): Boolean {
        if (waitFor.any { it.state == State.Cancelled }) requestStop(true)
        if (waitFor.any { it.state != State.Finished }) return false
        return true
    }
}

abstract class TaskTemplate(override val scheduler: Scheduler) : TaskWithWaitFor(), ITask {
    final override var state = State.NotStarted
    final override var myId: Int? = null
    override var name: String = "unnamed task"

    var startedAt = 0
        private set
    override var isStartRequested = false

    override fun transition(newState: State) {
        println("$this: transition: ${state.name} -> ${newState.name}")
        if (state.order > newState.order) {
            throw IllegalStateException("cannot move from ${state.name} to ${newState.name}")
        }
        if (state == newState) return
        when (newState) {
            State.Starting -> startedAt = scheduler.getTicks()
            State.Finishing -> println("$this: finishing at ${scheduler.getTicks()} (run for ${scheduler.getTicks() - (startedAt ?: 0)} ticks)")
            else -> {}
        }
        state = newState
    }

    override fun register() {
        myId = scheduler.register(this)
    }

    override fun requestStart() {
        isStartRequested = true
    }

    override fun requirements(): Set<SharedResource> = setOf()

    override fun invokeOnStart() {}

    override fun invokeOnTick() {}

    override fun invokeIsCompleted() = false

    override fun invokeOnFinish() {}

    override fun invokeCanStart() = super.invokeCanStart()

    override val daemon: Boolean = false
}
