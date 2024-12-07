package dev.aether.collaborative_multitasking

import dev.aether.collaborative_multitasking.TaskTemplate

enum class ResolveReject {
    RESOLVE,
    REJECT
}

interface ITaskWithResult<T> : ITask {
    fun getResultMaybe(): T?
    fun getResult(): T = getResultMaybe() ?: throw IllegalStateException("No result was available")
    fun hasResult(): Boolean

    fun <X : ITaskConsumer<T>> then(task: X): X {
        then(task as ITask)
        task.upstreamTyped(this)
        return task
    }
}

interface ITaskConsumer<T> : ITask {
    fun upstreamTyped(provider: ITaskWithResult<T>)
}


abstract class TaskWithResultTemplate<T>(scheduler: Scheduler) : TaskTemplate(scheduler), ITaskWithResult<T> {
    private var result: T? = null

    /**
     * Sets the result of this task. Only usable once per task execution.
     * Usually a good idea to put this in onFinished.
     */
    protected fun setResult(value: T) {
        if (result == null) {
            result = value
        } else throw IllegalStateException("A result was already assigned to this task")
    }

    override fun hasResult(): Boolean = result != null

    override fun getResultMaybe(): T? = result

    override fun then(configure: Task.() -> Unit): Task {
        return super<TaskTemplate>.then(configure)
    }

    override fun then(polyChain: Pair<Task, Task>): Task {
        return super<TaskTemplate>.then(polyChain)
    }
}