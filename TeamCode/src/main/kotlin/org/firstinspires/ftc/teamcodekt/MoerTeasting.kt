@file:ConfigKt

package org.firstinspires.ftc.teamcodekt

import ftc.rogue.blacksmith.annotations.ConfigKt
import ftc.rogue.blacksmith.internal.tryConfigKtSetup
import ftc.rogue.blacksmith.util.kt.clamp

@ConfigKt
object MoerTeasting {
    var hi = 3
}

var HELLO = 4.toInt().coerceAtLeast(2).clamp<Int>(0, 5)
var WHELLO = "4"
