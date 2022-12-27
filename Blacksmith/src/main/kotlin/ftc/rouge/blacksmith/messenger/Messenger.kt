package ftc.rouge.blacksmith.messenger

object Messenger {
    private val messages = mutableMapOf<Any, MutableList<Runnable>>()

    fun emit(message: Any) {
        messages[message]?.forEach { it.run() }
    }

    fun on(message: Any, callback: Runnable) {
        messages.getOrPut(message, ::ArrayList) += callback
    }
}
