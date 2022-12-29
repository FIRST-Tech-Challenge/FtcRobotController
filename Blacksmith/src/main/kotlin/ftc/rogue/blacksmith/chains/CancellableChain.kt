package ftc.rogue.blacksmith.chains

import ftc.rogue.blacksmith.listeners.Listener

interface CancellableChain : Chain {
    fun cancelOn(listener: Listener)
}
