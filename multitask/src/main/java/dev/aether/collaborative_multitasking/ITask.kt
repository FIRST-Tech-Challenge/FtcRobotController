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

    val state: State
    val myId: Int
    val name: String
    val daemon: Boolean
    fun onRequest(): Boolean = false

    // Lifecycle
    fun setState(newState: State)

    fun invokeCanStart(): Boolean
    fun invokeOnStart()
    fun invokeOnTick()
    fun invokeIsCompleted(): Boolean
    fun invokeOnFinish()

    fun requirements(): Set<SharedResource>
}
