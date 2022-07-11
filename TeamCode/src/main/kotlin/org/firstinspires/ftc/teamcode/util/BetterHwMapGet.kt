package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap

inline fun <reified T> HardwareMap._get(name: String): T = get(T::class.java, name)