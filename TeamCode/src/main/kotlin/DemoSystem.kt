package org.firstinspires.ftc.teamcode

/**
 * RoboHawks 5741 Demo System v1
 * Used to create an autonomous OpMode in around 2 hours.
 * It's a bit clunky, but we used it to "reprogram" our autonomous on the fly.
 * You can record TeleOp gameplay, then play it back as autonomous.
 * REQUIREMENTS:
 * - A project with Kotlin support (root buildscript dependency `org.jetbrains.kotlin:kotlin-gradle-plugin` and plugin `org.jetbrains.kotlin.android`)
 * - Kotlin's reflection library (org.jetbrains.kotlin:kotlin-reflect)
 * - A TeleOp OpMode written in Kotlin (you can convert Java to Kotlin in Android Studio/IntelliJ IDEA)
 */

/*
 * The Clear BSD License
 *
 * Copyright (c) 2023 RoboHawks 5741
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *
 *      * Neither the name of the copyright holder nor the names of its
 *      contributors may be used to endorse or promote products derived from this
 *      software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

import android.util.Base64
import android.util.Log
import com.google.gson.Gson
import com.google.gson.JsonArray
import com.google.gson.JsonNull
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.*
import java.io.BufferedReader
import java.io.File
import java.io.FileReader
import java.io.FileWriter
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.reflect.KClass
import kotlin.reflect.full.createInstance

@Suppress("unused")
object DemoSystem {

    /** Replace this with your own Driver Control. It must be a Kotlin class. */
    val PLAYBACK_OPMODE: KClass<*> = mechanumTest::class
    const val TICK_RATE: Double = 30.0
    const val DEMO_DIRECTORY: String = "demos"
    var outputFileName: String = "0.replay"
    var inputFileName: String = "0.replay"

    var frames1: ArrayList<ByteArray?> = arrayOfNulls<ByteArray?>(ceil(TICK_RATE * 30.0).toInt()).toCollection(ArrayList())
    var frames2: ArrayList<ByteArray?> = arrayOfNulls<ByteArray?>(ceil(TICK_RATE * 30.0).toInt()).toCollection(ArrayList())

    fun ByteArray.toBase64(): String = String(Base64.encode(this, Base64.DEFAULT or Base64.NO_WRAP))

    @Autonomous(name = "Play Recorded Demo", group = "DemoSystem")
    open class DemoPlayback : OpMode() {

        private var timeOffset: Double = 0.0
        private val emulatedOpMode: OpMode = PLAYBACK_OPMODE.createInstance() as OpMode

        final override fun init() {
            val context = hardwareMap.appContext
            val dir = File(context.filesDir, DEMO_DIRECTORY)
            if (!dir.exists()) {
                dir.mkdir()
            }
            val file = File(dir, inputFileName)
            val fileReader = FileReader(file)
            val reader = BufferedReader(fileReader)
            val lines = StringBuilder(file.length().toInt())
            while (true) {
                val line = reader.readLine()
                if (line == null) break; else lines.append(line)
            }
            val gson = Gson()
            val jsonAllFrames: Array<Array<String?>> = gson.fromJson(lines.toString(), Array<Array<String?>>::class.java)

//            var totalSize = 0
            val jsonFrames1 = jsonAllFrames[0]
            val jsonFrames2 = jsonAllFrames[1]
            for ((i, e) in jsonFrames1.withIndex()) frames1[i] = if (e is String) Base64.decode(e, Base64.DEFAULT or Base64.NO_WRAP) else null
            for ((i, e) in jsonFrames2.withIndex()) frames2[i] = if (e is String) Base64.decode(e, Base64.DEFAULT or Base64.NO_WRAP) else null

            println()

            emulatedOpMode.gamepad1 = Gamepad()
            emulatedOpMode.gamepad2 = Gamepad()
            emulatedOpMode.telemetry = this.telemetry
            emulatedOpMode.hardwareMap = this.hardwareMap
//            emulatedOpMode.internalOpModeServices = this.internalOpModeServices
            emulatedOpMode.time = this.time
            emulatedOpMode.init()
        }

        final override fun start() {
            this.timeOffset = this.time
            emulatedOpMode.time = this.time - timeOffset
            emulatedOpMode.start()
        }

        final override fun loop() {
            emulatedOpMode.time = this.time - timeOffset

            val index = floor((this.time - timeOffset) * TICK_RATE).toInt()

            if (index < frames1.size && frames1[index] != null) {
                emulatedOpMode.gamepad1.fromByteArray(frames1[index])
                emulatedOpMode.gamepad2.fromByteArray(frames2[index])
                telemetry.addLine("frame #${index}")
                Log.i("DemoSystem", "playing frame #$index")
            } else {
                Log.i("DemoSystem", "skipping frame read #$index (max frames = ${frames1.size}, frame@index = ${frames1[index]})")
            }

            emulatedOpMode.loop()
        }
    }

    @TeleOp(name = "Record Demo", group = "DemoSystem")
    class DemoRecorder : OpMode() {

        private var timeOffset: Double = 0.0
        private val emulatedOpMode: OpMode = PLAYBACK_OPMODE.createInstance() as OpMode

        override fun init() {
            frames1 = arrayOfNulls<ByteArray?>(ceil(TICK_RATE * 30.0).toInt()).toCollection(ArrayList())
            frames2 = arrayOfNulls<ByteArray?>(ceil(TICK_RATE * 30.0).toInt()).toCollection(ArrayList())

            emulatedOpMode.gamepad1 = Gamepad()
            emulatedOpMode.gamepad2 = Gamepad()
            emulatedOpMode.gamepad1.copy(this.gamepad1)
            emulatedOpMode.gamepad2.copy(this.gamepad2)
            frames1[0] = emulatedOpMode.gamepad1.toByteArray()
            frames2[0] = emulatedOpMode.gamepad2.toByteArray()

            emulatedOpMode.telemetry = this.telemetry
            emulatedOpMode.hardwareMap = this.hardwareMap
//            emulatedOpMode.internalOpModeServices = this.internalOpModeServices
            emulatedOpMode.time = this.time
            emulatedOpMode.init()
        }

        override fun start() {
            this.timeOffset = this.time
            emulatedOpMode.time = this.time - timeOffset
            emulatedOpMode.gamepad1.copy(this.gamepad1)
            emulatedOpMode.gamepad2.copy(this.gamepad2)
            emulatedOpMode.start()
            Log.i("DemoSystem", "Started recording.")
        }

        override fun loop() {
            emulatedOpMode.time = this.time - timeOffset
            emulatedOpMode.gamepad1.copy(this.gamepad1)
            emulatedOpMode.gamepad2.copy(this.gamepad2)

            val index = floor((this.time - timeOffset) * TICK_RATE).toInt()

            if (index < frames1.size && frames1[index] == null) {
                frames1[index] = emulatedOpMode.gamepad1.toByteArray()
                frames2[index] = emulatedOpMode.gamepad2.toByteArray()
                Log.i("DemoSystem", "recorded frame #$index")
            } else {
                Log.i("DemoSystem", "skipping frame write #$index (max frames = ${frames1.size}, frame@index = ${frames1[index]})")
            }

            emulatedOpMode.loop()
        }

        override fun stop() {
            Log.i("DemoSystem", "Demo recorded, frames:")
            for ((i, e) in frames1.withIndex()) {
                if (e == null) continue
                Log.i("DemoSystem", "#$i: $e")
            }
            Log.i("DemoSystem", "No more frames.")

            val context = hardwareMap.appContext
            val dir = File(context.filesDir, DEMO_DIRECTORY)
            if (!dir.exists()) {
                dir.mkdir()
            }
            val file = File(dir, outputFileName)
            val jsonAllFrames = JsonArray()
            val jsonFrames1 = JsonArray()
            val jsonFrames2 = JsonArray()
            val writer = FileWriter(file)
//            var totalSize = 0
            for (e in frames1) if (e == null) jsonFrames1.add(JsonNull.INSTANCE); else jsonFrames1.add(e.toBase64())
            for (e in frames2) if (e == null) jsonFrames2.add(JsonNull.INSTANCE); else jsonFrames2.add(e.toBase64())
            jsonAllFrames.add(jsonFrames1)
            jsonAllFrames.add(jsonFrames2)
            writer.write(jsonAllFrames.toString())
            writer.close()
            Log.i("DemoSystem", "Wrote demo to ${file.absolutePath}")
        }
    }
}
