package ftc.rogue.blacksmith.chains

import ftc.rogue.blacksmith.listeners.Listener

interface Chain {
    fun invokeOn(listener: Listener)
}