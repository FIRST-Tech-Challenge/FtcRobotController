package org.firstinspires.ftc.teamcodekt.blacksmith.chains

import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener

interface CancellableChain : Chain {
    fun cancelOn(listener: Listener)
}
