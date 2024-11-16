@file:Suppress("unused")

package dev.aether.collaborative_multitasking

import dev.aether.collaborative_multitasking.ITask.State

/*
@startuml
hide empty description
state Task {
    state "Waiting" as NotStarted
    state "Starting" as Starting {
        state "Event" as StartEvent : call start()
        state "isCompleted()" as StartIsCompleted : check for completion
        StartEvent -> StartIsCompleted
    }
    state "Ticking" as Ticking {
        state "Event" as TickEvent : call tick()
        state "isCompleted()" as TickIsCompleted : check for completion
        TickEvent -> TickIsCompleted
    }
    state "Finishing" as Finishing {
        state "Event" as FinishEvent : call finish()
        state "then(...)" as FinishAndThen : queue next tasks
        FinishEvent -> FinishAndThen
    }
    state "Finished" as Finished
    NotStarted <- [*]
    NotStarted --> Starting : canStart and\n scheduler choice
    Starting -> Ticking : not completed
    Starting --> Finishing : completed
    Ticking -> Ticking : scheduler tick
    Ticking --> Finishing : completed
    Finishing --> Finished
    Finished -> [*]
}
@enduml
 */

typealias TaskQuery2<T> = (task: Task, scheduler: Scheduler) -> T
typealias TaskQuery1<T> = (task: Task) -> T
typealias Producer<T> = () -> T
typealias TaskAction2 = TaskQuery2<Unit>
typealias TaskAction1 = TaskQuery1<Unit>
typealias Runnable = () -> Unit

class Task(
    override var scheduler: Scheduler,
): TaskWithWaitFor() {
    override var state: State = State.NotStarted
        private set

    var startedAt: Int? = null
        private set

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

    private var requirements: MutableSet<SharedResource> = mutableSetOf()

    override var isStartRequested = false
    override fun requestStart() {
        isStartRequested = true
    }

    internal var canStart: TaskQuery2<Boolean> = { _: Task, _: Scheduler -> true }
    internal var onStart: TaskAction2 = { _: Task, _: Scheduler -> }
    private var onTick: TaskAction2 = { _: Task, _: Scheduler -> }
    internal var isCompleted: TaskQuery2<Boolean> = { _: Task, _: Scheduler -> false }
    private var onFinish: TaskAction2 = { _: Task, _: Scheduler -> }
    private var then: TaskAction2 = { _: Task, _: Scheduler -> }

    override var name: String = "unnamed task"

    override var daemon = false
    override var myId: Int? = null

    fun canStart(block: TaskQuery2<Boolean>) {
        canStart = block
    }

    fun canStart(block: TaskQuery1<Boolean>) {
        canStart = { that: Task, _: Scheduler -> block(that) }
    }

    fun canStart(block: Producer<Boolean>) {
        canStart = { _: Task, _: Scheduler -> block() }
    }

    fun startOnRequest() = canStart(::onRequest)

    override fun invokeCanStart(): Boolean {
        return super.invokeCanStart() && canStart(this, scheduler)
    }

    fun onStart(block: TaskAction2) {
        onStart = block
    }

    fun onStart(block: TaskAction1) {
        onStart = { that: Task, _: Scheduler -> block(that) }
    }

    fun onStart(block: Runnable) {
        onStart = { _: Task, _: Scheduler -> block() }
    }

    override fun invokeOnStart() {
        onStart(this, scheduler)
    }

    fun onTick(block: TaskAction2) {
        onTick = block
    }

    fun onTick(block: TaskAction1) {
        onTick = { that: Task, _: Scheduler -> block(that) }
    }

    fun onTick(block: Runnable) {
        onTick = { _: Task, _: Scheduler -> block() }
    }

    override fun invokeOnTick() {
        onTick(this, scheduler)
    }

    fun isCompleted(block: TaskQuery2<Boolean>) {
        isCompleted = block
    }

    fun isCompleted(block: TaskQuery1<Boolean>) {
        isCompleted = { that: Task, _: Scheduler -> block(that) }
    }

    fun isCompleted(block: Producer<Boolean>) {
        isCompleted = { _: Task, _: Scheduler -> block() }
    }

    override fun invokeIsCompleted(): Boolean {
        return isCompleted(this, scheduler)
    }

    fun onFinish(block: TaskAction2) {
        onFinish = block
    }

    fun onFinish(block: TaskAction1) {
        onFinish = { that: Task, _: Scheduler -> block(that) }
    }

    fun onFinish(block: Runnable) {
        onFinish = { _: Task, _: Scheduler -> block() }
    }

    override fun invokeOnFinish() {
        onFinish(this, scheduler)
    }

    operator fun SharedResource.unaryPlus() {
        require(this)
    }

    private fun unrequire(lockName: SharedResource) {
        requirements.remove(lockName)
    }

    fun require(lockName: SharedResource) {
        requirements.add(lockName)
    }

    override fun requirements(): Set<SharedResource> {
        return requirements.toSet()
    }

    override fun register() {
        myId = scheduler.register(this)
    }

    fun apply(configure: Task.() -> Unit) {
        this.configure()
    }

    override fun toString(): String {
        return "task $myId '$name'"
    }

    override fun hashCode() = (myId ?: 0) * name.hashCode()
    override fun equals(other: Any?) = this === other
}
