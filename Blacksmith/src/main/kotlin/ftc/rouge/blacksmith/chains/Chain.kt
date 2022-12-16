package ftc.rouge.blacksmith.chains

import ftc.rouge.blacksmith.listeners.Listener

interface Chain {
    fun invokeOn(listener: Listener)
}