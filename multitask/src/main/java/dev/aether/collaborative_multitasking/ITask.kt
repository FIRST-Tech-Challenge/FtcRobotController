package dev.aether.collaborative_multitasking

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
    val name: String
    val daemon: Boolean
    val isStartRequested: Boolean
    fun onRequest() = isStartRequested

    // Lifecycle
    fun transition(newState: State)

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

    fun then(task: Task): Task {
        task waitsFor this
        return task
    }

    fun then(polyChain: Pair<Task, Task>): Task {
        this.then(polyChain.first)
        return polyChain.second
    }
}

abstract class TaskWithWaitFor(): ITask {

    private var waitFor: MutableSet<ITask> = mutableSetOf()

    fun getWaitsFor() = waitFor as Set<ITask>

    override fun waitsFor(after: ITask) {
        waitFor.add(after)
    }

    override fun invokeCanStart(): Boolean {
        if (waitFor.any { it.state == ITask.State.Cancelled }) requestStop(true)
        if (waitFor.any { it.state != ITask.State.Finished }) return false
        return true
    }
}
