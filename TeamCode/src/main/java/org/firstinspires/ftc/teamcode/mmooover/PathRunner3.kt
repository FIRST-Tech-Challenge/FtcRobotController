package org.firstinspires.ftc.teamcode.mmooover

import android.os.Environment
import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.mmooover.kinematics.Waypoint3
import org.firstinspires.ftc.teamcode.mmooover.kinematics.WaypointSerializer
import java.io.DataInputStream
import java.io.File
import java.io.InputStream

class PathRunner3(filepath: File) {
    companion object {
        fun getPathfileByName(name: String): File {
            return Environment.getExternalStorageDirectory().resolve("paths").resolve("$name.bin")
        }
    }

    // Intentionally immutable.
    val waypoints: List<Waypoint3>

    init {
        if (!filepath.exists()) throw IllegalArgumentException("The file specified doesn't exist")
        Log.i("PathRunner3", "Loading waypoints from $filepath, hold on...")
        waypoints = filepath.inputStream().use { fileIn ->
            DataInputStream(fileIn).use { dataIn ->
                WaypointSerializer.deserialize3(dataIn)
            }
        }
        Log.i("PathRunner3", "Loaded ${waypoints.size} waypoints.")
    }
}