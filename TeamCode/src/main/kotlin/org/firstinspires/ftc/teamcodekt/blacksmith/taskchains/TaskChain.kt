package org.firstinspires.ftc.teamcodekt.blacksmith.taskchains

import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener

interface TaskChain {
    fun invokeOn(listener: Listener)
}