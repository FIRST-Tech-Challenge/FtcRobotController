package org.firstinspires.ftc.teamcode.mmooover.kinematics

import java.io.DataInputStream
import java.io.DataOutputStream

object CommandSerializer {
    @JvmStatic
    fun serialize(units: List<BytecodeUnit>, target: DataOutputStream) {
        target.writeBytes("PC")
        target.writeInt(units.size)
        for (unit in units) {
            unit.export(target)
        }
    }

    @JvmStatic
    fun deserialize(target: DataInputStream): List<BytecodeUnit> {
        assert(
            target.readByte() == 'P'.code.toByte() && target.readByte() == 'C'.code.toByte()
        ) { "Not a command file (invalid magic)" }
        val size = target.readInt()
        val result: MutableList<BytecodeUnit> = mutableListOf()
        repeat(size) {
            result.add(BytecodeUnit.import(target))
        }
        return result
    }
}