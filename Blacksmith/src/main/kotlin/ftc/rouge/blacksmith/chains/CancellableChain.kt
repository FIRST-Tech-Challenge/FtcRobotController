package ftc.rouge.blacksmith.chains

import ftc.rouge.blacksmith.listeners.Listener

interface CancellableChain : Chain {
    fun cancelOn(listener: Listener)
}
