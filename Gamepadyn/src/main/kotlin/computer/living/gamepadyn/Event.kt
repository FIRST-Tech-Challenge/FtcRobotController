package computer.living.gamepadyn

/**
 * An instance of an Event represents *one event* to which listeners are added and called when the event is triggered.
 */
class Event<T> {
    private val listeners = mutableSetOf<((T) -> Unit)>();

    /**
     * Adds a callback for the event.
     * @return true if it was added and false if it was already there before
     */
    fun addListener(listener: ((T) -> Unit)): Boolean = listeners.add(listener)

    /**
     * Removes an already-present callback for the event.
     * @return true if it was removed and false if it wasn't a listener before
     */
    fun removeListener(listener: ((T) -> Unit)): Boolean = listeners.remove(listener)

    /**
     * Removes all listeners from the event.
     */
    fun clearListeners(): Unit = listeners.clear()

    internal fun trigger(data: T) {
        for (e in listeners) e.invoke(data);
    }

}