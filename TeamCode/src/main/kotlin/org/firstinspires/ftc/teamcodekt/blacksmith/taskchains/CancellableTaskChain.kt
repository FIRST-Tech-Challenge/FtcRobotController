package org.firstinspires.ftc.teamcodekt.blacksmith.taskchains

import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener

interface CancellableTaskChain : TaskChain {
    fun cancelOn(listener: Listener)
}
