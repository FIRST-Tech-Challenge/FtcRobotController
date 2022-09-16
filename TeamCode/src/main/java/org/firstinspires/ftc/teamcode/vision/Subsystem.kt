package org.firstinspires.ftc.teamcode.vision

interface Subsystem {
    fun update()
    fun sendDashboardPacket(debugging: Boolean)
    fun reset()
}