package org.firstinspires.ftc.teamcodekt.blacksmith.chains

import org.firstinspires.ftc.teamcodekt.blacksmith.listeners.Listener

interface Chain {
    fun invokeOn(listener: Listener)
}