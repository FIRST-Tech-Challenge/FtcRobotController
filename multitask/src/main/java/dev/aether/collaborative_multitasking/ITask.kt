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

    var scheduler: Scheduler
    val state: State
    val myId: Int?
    var name: String
    val daemon: Boolean
    val isStartRequested: Boolean
    fun onRequest() = isStartRequested

    fun isBypass() = false

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
        scheduler.filteredStop({ it === this }, cancel)
    }

    fun requestStop() = requestStop(true)

    fun then(configure: Task.() -> Unit): Task {
        val task = Task(scheduler)
        task.name = getCaller()
        task.configure()
        task waitsFor this
        task.register() // ready to go
        return task
    }

    fun <T : ITask> then(task: T): T {
        task.scheduler = scheduler
        task waitsFor this
        task.register()
        return task
    }

    fun then(polyChain: Pair<ITask, ITask>): ITask {
        this.then(polyChain.first)
        return polyChain.second
    }

    fun display(indent: Int, write: (String) -> Unit) {}
}

abstract class TaskWithChaining() : ITask {

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

abstract class TaskTemplate(override var scheduler: Scheduler) : TaskWithChaining(), ITask {
    final override var state = State.NotStarted
    final override var myId: Int? = null
    private var name2 = "unnamed task"
    override var name: String
        get() = name2
        set(value) {
            name2 = value
        }

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
            State.Finishing -> println("$this: finishing at ${scheduler.getTicks()} (run for ${scheduler.getTicks() - startedAt} ticks)")
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

    override fun toString(): String {
        return "task $myId '$name'"
    }
}

abstract class ConsumingTaskTemplate<T>(scheduler: Scheduler) : TaskTemplate(scheduler),
    ITaskConsumer<T> {
    private val typedUpstreams: MutableList<ITaskWithResult<T>> = mutableListOf()

    override fun upstreamTyped(provider: ITaskWithResult<T>) {
        typedUpstreams.add(provider)
    }

    protected val weakResult: T?
        get() = nullableFindReadyUpstream(typedUpstreams)
    protected val result: T
        get() = findReadyUpstream(typedUpstreams)
}

private fun <T> nullableFindReadyUpstream(providers: List<ITaskWithResult<T>>): T? {
    var result: T? = null
    for (item in providers) {
        item.getResultMaybe().let {
            if (result != null) throw IllegalStateException("More than one upstream result is available")
            result = it
        }
    }
    return result
}

private fun <T> findReadyUpstream(providers: List<ITaskWithResult<T>>): T =
    nullableFindReadyUpstream(providers)
        ?: throw IllegalStateException("No upstream results available yet")
