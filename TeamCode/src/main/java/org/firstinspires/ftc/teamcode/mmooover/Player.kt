package org.firstinspires.ftc.teamcode.mmooover

import android.os.Environment
import android.util.Log
import org.firstinspires.ftc.teamcode.mmooover.kinematics.BytecodeUnit
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CommandSerializer
import java.io.DataInputStream
import java.io.File

class Player(filepath: File) {
    companion object {
        fun getPathfileByName(name: String): File {
            return Environment.getExternalStorageDirectory().resolve("paths").resolve("$name.bin")
        }
    }

    // Intentionally immutable.
    val commands: List<BytecodeUnit>

    init {
        if (!filepath.exists()) throw IllegalArgumentException("The file specified doesn't exist")
        Log.i("PathRunner3", "Loading waypoints from $filepath, hold on...")
        commands = filepath.inputStream().use { fileIn ->
            DataInputStream(fileIn).use { dataIn ->
                CommandSerializer.deserialize(dataIn)
            }
        }
        Log.i("PathRunner3", "Loaded ${commands.size} waypoints.")
    }
}