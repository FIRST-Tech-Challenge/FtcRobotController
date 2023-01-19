package ftc.rogue.blacksmith.chains

import ftc.rogue.blacksmith.listeners.Listener

fun interface Chain {
    fun invokeOn(listener: Listener)
}
