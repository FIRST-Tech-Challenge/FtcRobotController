package org.firstinspires.ftc.teamcode.subsystem

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class Limelight(
    taskManager: TaskManager,
    val limelight3A: Limelight3A
) : Subsystem() {
    companion object {
        const val PIPELINE_ID: Int = 0
        const val POLL_RATE: Int = 20
        private fun isUpsideDown(result: FiducialResult): Boolean {
            return result.targetPoseCameraSpace.orientation.getRoll(AngleUnit.DEGREES) > 180
        }
    }
    enum class Mode {
        ON,
        OFF
    }
    var mode: Mode = Mode.OFF
        set(mode) {
            field = mode
            limelight3A.apply {
                when (mode) {
                    Mode.ON -> {
                        pipelineSwitch(PIPELINE_ID)
                        setPollRateHz(POLL_RATE)
                        start()
                    }
                    Mode.OFF -> {
                        stop()
                    }
                }
            }
        }
    init {
        taskManager.loop.init += { updateHiveOrientations() }
    }

    fun updateHiveOrientations() {
        if (mode == Mode.OFF) return
        val results = limelight3A.latestResult.fiducialResults
        for (result in results) {
            Hive[result.fiducialId]?.apply {
                alliance.orientation = when {
                    isUpsideDown(result) -> !orientation
                    else -> orientation
                }
            }
        }
    }

    override fun printState(telemetry: Telemetry) {
        TODO("Not yet implemented")
    }
}
