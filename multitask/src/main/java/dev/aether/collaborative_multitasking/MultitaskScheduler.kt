@file:Suppress("DefaultLocale")

package dev.aether.collaborative_multitasking

import kotlin.math.ceil
import kotlin.math.max

class MultitaskScheduler
@JvmOverloads constructor(private val throwDebuggingErrors: Boolean = false) : Scheduler() {
    private val locks: MutableMap<String, Int?> = mutableMapOf()
    private val tasks: MutableMap<Int, Task> = mutableMapOf()
    private val lockIdName: MutableMap<String, SharedResource> = mutableMapOf()

    companion object {
        const val evict = true
        const val rollingAverageSize = 10000

        private fun <T : Comparable<T>, N : List<T>> percentile(k: N, l: Double): T {
            val index = ceil(l * k.size).toInt()
            return k[max(0, index - 1)]
        }

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
    }

    val tickTimes: MutableList<Double> = mutableListOf()

    override var nextId: Int = 0
        private set
    private var tickCount = 0

    private fun selectState(state: Task.State): List<Task> {
        return tasks.values.filter { it.state == state }
    }

    private fun allFreed(requirements: Set<SharedResource>): Boolean {
        return requirements.all { locks[it.id] == null }
    }

    private fun tickMarkStartable() {
        selectState(Task.State.NotStarted)
            .filter {
                it.invokeCanStart()
            }
            .forEach {
                if (allFreed(it.requirements())) {
                    it.setState(Task.State.Starting)
                    // acquire locks
                    for (lock in it.requirements()) {
                        println("$it acquired $lock")
                        if (locks[lock.id] != null) {
                            println("WARN: uhh we're gonna make ${tasks[locks[lock.id]]} crash when it finishes")
                        }
                        locks[lock.id] = it.myId
                        lockIdName[lock.id] = lock
                        println("locks: $locks")
                    }
                }
            }
    }

    private fun tickStartMarked() {
        selectState(Task.State.Starting)
            .forEach {
                try {
                    it.invokeOnStart()
                    if (it.invokeIsCompleted()) {
                        it.setState(Task.State.Finishing)
                    } else {
                        it.setState(Task.State.Ticking)
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

    private fun tickTick() {
        selectState(Task.State.Ticking)
            .forEach {
                try {
                    it.invokeOnTick()
                    if (it.invokeIsCompleted()) it.setState(Task.State.Finishing)
                } catch (e: Exception) {
                    System.err.println(String.format("Error while ticking %s:", it.toString()))
                    e.printStackTrace()
                }
            }
    }

    private fun tickFinish() {
        val candidates = selectState(Task.State.Finishing)
        candidates.forEach(::release)
    }

    private fun release(task: Task, cancel: Boolean = false) {
        val targetState = if (cancel) Task.State.Cancelled else Task.State.Finished
        if (task.state == Task.State.NotStarted) {
            task.setState(targetState)
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
        task.setState(targetState)
        for (lock in task.requirements()) {
            if (locks[lock.id] != task.myId) {
                if (throwDebuggingErrors)
                    throw IllegalStateException("$task (which just finished) does not own lock $lock that it is supposed to own")
                else println("ERROR!!! $task (which just finished) does not own lock $lock that it is supposed to own")
            }
            locks[lock.id] = null
            println("$task released $lock")
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
                Task.State.NotStarted -> waiting++
                Task.State.Finished -> done++
                Task.State.Cancelled -> cancelled++
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

    private fun displayTaskNoStatus(task: Task, indent: Int, writeLine: (String) -> Unit) {
        writeLine(buildString {
            append(" ".repeat(indent))
            append("#%d (%s)".format(task.myId, task.name))
            if(task.daemon) append(" daemon")
            if(task.onRequest()) append(" startReq")
        })
    }

    fun displayStatus(withFinished: Boolean, withNotStarted: Boolean, writeLine: (String) -> Unit) {
        val notStartedList: MutableList<Task> = mutableListOf()
        val inProgressList: MutableList<Task> = mutableListOf()
        val finishedList: MutableList<Task> = mutableListOf()
        val cancelledList: MutableList<Task> = mutableListOf()
        var waiting = 0
        var progress = 0
        var done = 0
        var cancelled = 0
        var total = tasks.size
        for (task in tasks.values) {
            when (task.state) {
                Task.State.NotStarted -> {
                    waiting++
                    notStartedList.add(task)
                }
                Task.State.Finished -> {
                    done++
                    finishedList.add(task)
                }
                Task.State.Cancelled -> {
                    cancelled++
                    cancelledList.add(task)
                }
                else -> {
                    progress++
                    inProgressList.add(task)
                }
            }
        }
        writeLine("%d tasks: %d WAIT > %d RUN > %d STOP (%d done, %d cancel)".format(total, waiting, progress, done + cancelled, done, cancelled))
        if (withNotStarted) {
            writeLine("%d not started:".format(waiting))
            for (task in notStartedList) displayTaskNoStatus(task, 4, writeLine)
        }
        writeLine("%d in progress:".format(progress))
        for (task in inProgressList) displayTaskNoStatus(task, 4, writeLine)
        if (withFinished) {
            writeLine("%d finished:".format(done))
            for (task in finishedList) displayTaskNoStatus(task, 4, writeLine)
            writeLine("%d cancelled:".format(cancelled))
            for (task in cancelledList) displayTaskNoStatus(task, 4, writeLine)
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

    override fun register(task: Task): Int {
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

    override fun panic() {
        for (task in tasks.values) {
            if (task.state == Task.State.Finished || task.state == Task.State.NotStarted || task.state == Task.State.Cancelled) continue
            task.invokeOnFinish()
            task.setState(Task.State.Finished)
        }

        for (lock in lockIdName.values) {
            lock.panic()
        }
    }

    fun hasJobs(): Boolean {
        return tasks.values.any { it.state != Task.State.Finished && !it.daemon }
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
        predicate: (Task) -> Boolean,
        cancel: Boolean,
        dropNonStarted: Boolean
    ) {
        tasks.values
            .filter { it.state != Task.State.Finished && it.state != Task.State.NotStarted && it.state != Task.State.Cancelled }
            .filter(predicate)
            .forEach {
                release(it, cancel)
            }
        if (dropNonStarted) {
            val dropped = tasks.filterInPlace { k, v ->
                v.state == Task.State.NotStarted && predicate(v)
            }
            println("dropped ${dropped.size} tasks: ${dropped.joinToString(", ")}")
        }
    }

    override fun filteredStop(predicate: (Task) -> Boolean, cancel: Boolean) = filteredStop(
        predicate,
        cancel = cancel, dropNonStarted = false
    )

    override fun filteredStop(predicate: (Task) -> Boolean) = filteredStop(
        predicate,
        cancel = true,
        dropNonStarted = false
    )
}
