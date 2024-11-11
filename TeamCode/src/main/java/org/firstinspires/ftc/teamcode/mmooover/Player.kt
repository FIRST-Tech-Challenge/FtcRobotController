package org.firstinspires.ftc.teamcode.mmooover

import android.os.Environment
import android.util.Log
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.ITask.State
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskWithWaitFor
import org.firstinspires.ftc.teamcode.Hardware
import org.firstinspires.ftc.teamcode.mmooover.kinematics.AwaitCommand
import org.firstinspires.ftc.teamcode.mmooover.kinematics.BytecodeUnit
import org.firstinspires.ftc.teamcode.mmooover.kinematics.CommandSerializer
import org.firstinspires.ftc.teamcode.mmooover.kinematics.MoveCommand
import org.firstinspires.ftc.teamcode.mmooover.kinematics.RunAsyncCommand
import org.firstinspires.ftc.teamcode.mmooover.kinematics.RunCommand
import org.firstinspires.ftc.teamcode.utilities.LoopStopwatch
import java.io.DataInputStream
import java.io.File

class Player(
    filepath: File,
    override val scheduler: Scheduler,
    val encoders: EncoderTracking,
    val eventHandlers: Map<String, () -> ITask>,
) : TaskWithWaitFor() {
    companion object {
        fun getPathfileByName(name: String): File {
            return Environment.getExternalStorageDirectory().resolve("paths").resolve("$name.bin")
        }
    }

    operator fun invoke(configure: Player.() -> Unit): Player {
        this.configure()
        return this
    }

    // Intentionally immutable.
    val commands: List<BytecodeUnit>

    // Task-y things
    override var state = State.NotStarted
    override var myId: Int? = null
    override val name = "[Player of ${filepath.name}]"
    override val daemon = false

    var startedAt: Int? = null
        private set
    var done = false

    override fun transition(newState: State) {
        println("$this: transition: ${state.name} -> ${newState.name}")
        if (state.order > newState.order) {
            throw IllegalStateException("cannot move from ${state.name} to ${newState.name}")
        }
        if (state == newState) return
        when (newState) {
            State.Starting -> startedAt = scheduler.getTicks()
            State.Finishing -> println("$this: finishing at ${scheduler.getTicks()} (run for ${scheduler.getTicks() - (startedAt ?: 0)} ticks)")
            else -> {}
        }
        state = newState
    }

    override fun invokeOnStart() {
        clockTimer.clear()
    }

    override fun invokeOnTick() {
        updateData()
        if (!done)
            doIt()
    }

    override fun invokeIsCompleted(): Boolean {
        // Completed when all the waypoints are completed.
        // TODO: or not.
        return done
    }

    override fun invokeOnFinish() {}

    private val requires = setOf(
        Hardware.Locks.DriveMotors
    )

    override fun requirements(): Set<SharedResource> = requires

    override var isStartRequested = false

    override fun invokeCanStart(): Boolean {
        return super.invokeCanStart() && isStartRequested
    }

    override fun requestStart() {
        isStartRequested = true
    }

    var cursor = 0
    var currentCommand: BytecodeUnit
    var nextCommand: BytecodeUnit? = null
    val clockTimer = LoopStopwatch()

    init {
        if (!filepath.exists()) throw IllegalArgumentException("The file specified doesn't exist.")
        Log.i("PathRunner3", "Loading waypoints from $filepath, hold on...")
        commands = filepath.inputStream().use { fileIn ->
            DataInputStream(fileIn).use { dataIn ->
                CommandSerializer.deserialize(dataIn)
            }
        }
        if (commands.isEmpty()) throw IllegalArgumentException("The file specified contains no commands.")

        cursor = 0
        currentCommand = commands[0]
        nextCommand = commands.getOrNull(1)

        Log.i("PathRunner3", "Loaded ${commands.size} waypoints.")
    }

    /**
     * Things to run every frame.
     */
    fun updateData() {
        encoders.step()
        clockTimer.click()
    }

    fun doMove(command: MoveCommand) {
        val targetPose = command.pose
        val currentPose = encoders.getPose()
        val linDist = currentPose.linearDistanceTo(targetPose)
    }

    fun doIt() {
        val command = currentCommand
        return when (command) {
            is AwaitCommand -> TODO()
            is MoveCommand -> doMove(command)
            is RunAsyncCommand -> TODO()
            is RunCommand -> TODO()
        }
    }

    fun nextCommand() {
        currentCommand = commands[++cursor]
        nextCommand = commands.getOrNull(cursor + 1)
    }

    override fun toString(): String {
        return "task $myId '$name'"
    }
}