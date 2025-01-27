@file:Suppress("DefaultLocale")

package dev.aether.collaborative_multitasking

import kotlin.math.ceil
import kotlin.math.max
import dev.aether.collaborative_multitasking.ITask.State

internal fun getCaller(): String {
    try {
        throw Exception()
    } catch (e: Exception) {
        val stack = e.stackTrace
        for (frame in stack) {
            if (frame.className.contains("dev.aether.collaborative_multitasking")) continue
            return "${
                frame.className.split(".").last()
            }.${frame.methodName} line ${frame.lineNumber}"
        }
    }
    return "<unknown source>"
}

open class MultitaskScheduler
@JvmOverloads constructor(private val throwDebuggingErrors: Boolean = false) : Scheduler() {
    private val locks: MutableMap<String, Int?> = mutableMapOf()
    private val tasks: MutableMap<Int, ITask> = mutableMapOf()
    private val lockIdName: MutableMap<String, SharedResource> = mutableMapOf()

    companion object {
        const val evict = true
        const val rollingAverageSize = 10000

        private fun <T : Comparable<T>, N : List<T>> percentile(k: N, l: Double): T {
            val index = ceil(l * k.size).toInt()
            return k[max(0, index - 1)]
        }
    }

    val tickTimes: MutableList<Double> = mutableListOf()

    final override var nextId: Int = 0
        private set
    private var tickCount = 0

    protected fun selectState(state: State): List<ITask> {
        return tasks.values.filter { it.state == state }
    }

    protected fun allFreed(requirements: Set<SharedResource>): Boolean {
        return requirements.all { locks[it.id] == null }
    }

    protected fun tickMarkStartable() {
        selectState(State.NotStarted)
            .filter {
                it.invokeCanStart()
            }
            .forEach {
                if (it.isBypass()) {
                    val idsToStop = it.requirements().map { locks[it.id] }.filterNotNull()
                    filteredStop { it.myId in idsToStop }
                }
                if (allFreed(it.requirements())) {
                    it.transition(State.Starting)
                    // acquire locks
                    for (lock in it.requirements()) {
//                        println("$it acquired $lock")
                        if (locks[lock.id] != null) {
                            println("WARN: uhh we're gonna make ${tasks[locks[lock.id]]} crash when it finishes")
                        }
                        locks[lock.id] = it.myId
                        lockIdName[lock.id] = lock
//                        println("locks: $locks")
                    }
                }
            }
    }

    protected fun tickStartMarked() {
        selectState(State.Starting)
            .forEach {
                try {
                    it.invokeOnStart()
                    if (it.state != State.Starting) // cancelled
                        return@forEach
                    if (it.invokeIsCompleted()) {
                        it.transition(State.Finishing)
                    } else {
                        it.transition(State.Ticking)
                    }
                } catch (e: Exception) {
                    System.err.println(
                        String.format(
                            "Error while marking %s to start:",
                            it.toString()
                        )
                    )
                    e.printStackTrace()
                }
            }
    }

    protected fun tickTick() {
        selectState(State.Ticking)
            .forEach {
                try {
                    it.invokeOnTick()
                    if (it.invokeIsCompleted()) it.transition(State.Finishing)
                } catch (e: Exception) {
                    System.err.println(String.format("Error while ticking %s:", it.toString()))
                    e.printStackTrace()
                }
            }
    }

    protected fun tickFinish() {
        val candidates = selectState(State.Finishing)
        candidates.forEach(::release)
    }

    protected fun release(task: ITask, cancel: Boolean = false) {
        val targetState = if (cancel) State.Cancelled else State.Finished
        if (task.state == State.NotStarted) {
            task.transition(targetState)
            return
        }
        try {
            task.invokeOnFinish()
        } catch (e: Exception) {
            System.err.println(
                String.format(
                    "Error while processing %s finish handler:",
                    task.toString()
                )
            )
            e.printStackTrace()
        }
        task.transition(targetState)
        for (lock in task.requirements()) {
            if (locks[lock.id] != task.myId) {
                if (throwDebuggingErrors)
                    throw IllegalStateException("$task (which just finished) does not own lock $lock that it is supposed to own")
                else println("ERROR!!! $task (which just finished) does not own lock $lock that it is supposed to own")
            }
            locks[lock.id] = null
//            println("$task released $lock")
        }
    }

    override fun tick() {
        val start = System.nanoTime()
        tickMarkStartable()
        tickStartMarked()
        tickTick()
        tickFinish()
        tickCount++
        val durat = (System.nanoTime() - start) / 1000000.0
        tickTimes.add(durat)
        if (durat > 1000) {
            System.err.println(String.format("Warning: tick %d took %.2f ms", tickCount - 1, durat))
        }
        if (evict && tickTimes.size > rollingAverageSize) tickTimes.removeAt(0)
    }

    fun statsheet(): String {
        var waiting = 0
        var progress = 0
        var done = 0
        var cancelled = 0
        for (task in tasks.values) {
            when (task.state) {
                State.NotStarted -> waiting++
                State.Finished -> done++
                State.Cancelled -> cancelled++
                else -> progress++
            }
        }

        val s = tickTimes.sorted()
        return String.format(
            "${tasks.size} tasks: $waiting waiting, $progress running, $done done, $cancelled cancel\n" +
                    "%d samples:\n" +
                    "  [Min  ][1%%   ][5%%   ][32%%  ][50%%  ][68%%  ][95%%  ][99%%  ][Max  ]\n" +
                    "  %7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f%7.1f",
            s.size,
            percentile(s, 0.0),
            percentile(s, 0.01),
            percentile(s, 0.05),
            percentile(s, 0.32),
            percentile(s, 0.5),
            percentile(s, 0.68),
            percentile(s, 0.95),
            percentile(s, 0.99),
            percentile(s, 1.0)
        )
    }

    protected fun displayTaskNoStatus(task: ITask, indent: Int, writeLine: (String) -> Unit) {
        writeLine(buildString {
            append(" ".repeat(indent))
            append("#%d (%s)".format(task.myId, task.name))
            if (task.daemon) append(" daemon")
            if (task.onRequest()) append(" startReq")
        })
        task.display(indent + 4, writeLine)
    }

    @JvmOverloads
    fun displayStatus(
        withFinished: Boolean,
        withNotStarted: Boolean,
        writeLine: (String) -> Unit,
        indent: Int = 0
    ) {
        val notStartedList: MutableList<ITask> = mutableListOf()
        val inProgressList: MutableList<ITask> = mutableListOf()
        val finishedList: MutableList<ITask> = mutableListOf()
        val cancelledList: MutableList<ITask> = mutableListOf()
        var waiting = 0
        var progress = 0
        var done = 0
        var cancelled = 0
        var total = tasks.size

        val spaces = " ".repeat(indent)

        for (task in tasks.values) {
            when (task.state) {
                State.NotStarted -> {
                    waiting++
                    notStartedList.add(task)
                }

                State.Finished -> {
                    done++
                    finishedList.add(task)
                }

                State.Cancelled -> {
                    cancelled++
                    cancelledList.add(task)
                }

                else -> {
                    progress++
                    inProgressList.add(task)
                }
            }
        }
        writeLine(
            "%s%d tasks: %d WAIT > %d RUN > %d STOP (%d done, %d cancel)".format(
                spaces,
                total,
                waiting,
                progress,
                done + cancelled,
                done,
                cancelled
            )
        )
        if (withNotStarted) {
            writeLine("%s%d not started:".format(spaces, waiting))
            for (task in notStartedList) displayTaskNoStatus(task, indent + 4, writeLine)
        }
        writeLine("%s%d in progress:".format(spaces, progress))
        for (task in inProgressList) displayTaskNoStatus(task, indent + 4, writeLine)
        if (withFinished) {
            writeLine("%s%d finished:".format(spaces, done))
            for (task in finishedList) displayTaskNoStatus(task, indent + 4, writeLine)
            writeLine("%s%d cancelled:".format(spaces, cancelled))
            for (task in cancelledList) displayTaskNoStatus(task, indent + 4, writeLine)
        }
    }

    override fun getTicks(): Int {
        return tickCount
    }

    override fun task(configure: Task.() -> Unit): Task {
        val task = Task(this)
        task.configure()
        task.name = getCaller()
        task.register()
        return task
    }

    override fun <T : ITask> add(t: T): T {
        t.scheduler = this
        t.name = getCaller()
        t.register()
        return t
    }

    override fun register(task: ITask): Int {
        val id = nextId++
        tasks[id] = task
        for (lock in task.requirements()) {
            if (!locks.containsKey(lock.id)) {
                locks[lock.id] = null
            }
        }
        return id
    }

    override fun isResourceInUse(resource: SharedResource): Boolean {
        return locks[resource.id] != null
    }

    override fun manualAcquire(resource: SharedResource): Boolean {
        if (locks[resource.id] != null) return false
        locks[resource.id] = -1
        return true
    }

    override fun manualRelease(resource: SharedResource) {
        if (locks[resource.id] != -1) {
            if (throwDebuggingErrors) throw IllegalStateException("Cannot manually release: lock not held by [-1], or was already released")
            else println("ERROR [suppressed]: Cannot manually release: lock not held by [-1], or was already released")
            return
        }
        locks[resource.id] = null
    }

    override fun panic() {
        for (task in tasks.values) {
            if (task.state == State.Finished || task.state == State.NotStarted || task.state == State.Cancelled) continue
            task.invokeOnFinish()
            task.transition(State.Finished)
        }

        for (lock in lockIdName.values) {
            lock.panic()
        }
    }

    fun hasJobs(): Boolean {
        return tasks.values.any { it.state != State.Finished && !it.daemon }
    }

    fun runToCompletion(ok: () -> Boolean) {
        while (hasJobs() && ok()) {
            tick()
        }
    }

    /**
     * Stops any tasks matching the predicate that are not already finished or haven't started yet.
     * Resources owned by matching tasks are guaranteed to be released after this call.
     */

    override fun filteredStop(
        predicate: (ITask) -> Boolean,
        cancel: Boolean,
        dropNonStarted: Boolean
    ) {
        tasks.values
            .filter { it.state != State.Finished && it.state != State.NotStarted && it.state != State.Cancelled }
            .filter(predicate)
            .forEach {
                release(it, cancel)
            }
        if (dropNonStarted) {
            val dropped = tasks.filterInPlace { k, v ->
                !(v.state == State.NotStarted && predicate(v))
            }
//            println("dropped ${dropped.size} tasks: ${dropped.joinToString(", ")}")
        }
    }

    override fun filteredStop(predicate: (ITask) -> Boolean, cancel: Boolean) = filteredStop(
        predicate,
        cancel = cancel, dropNonStarted = false
    )

    override fun filteredStop(predicate: (ITask) -> Boolean) = filteredStop(
        predicate,
        cancel = true,
        dropNonStarted = false
    )

    override fun taskCount(): Int = tasks.size
}
