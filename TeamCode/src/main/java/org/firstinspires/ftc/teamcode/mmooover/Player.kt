package org.firstinspires.ftc.teamcode.mmooover

import android.os.Environment
import android.util.Log
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.SharedResource
import dev.aether.collaborative_multitasking.TaskTemplate
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
import java.lang.Math.toRadians

abstract class Configurable<T : Configurable<T>> {
    @Suppress("UNCHECKED_CAST")
    inline operator fun invoke(configure: T.() -> Unit) {
        (this as T).configure()
    }
}

class Player(
    filepath: File,
    scheduler: Scheduler,
    val encoders: EncoderTracking,
    val eventHandlers: Map<String, () -> ITask>,
    val options: PlayerConfiguration
) : TaskTemplate(scheduler) {

    constructor(
        filepath: File,
        scheduler: Scheduler,
        encoders: EncoderTracking,
        eventHandlers: Map<String, () -> ITask>,
        configure: PlayerConfiguration.() -> Unit
    ): this(filepath, scheduler, encoders, eventHandlers, PlayerConfiguration.of(configure))

    companion object {
        fun getPathfileByName(name: String): File {
            return Environment.getExternalStorageDirectory().resolve("paths").resolve("$name.bin")
        }
    }

    val Number.deg get() = toRadians(this.toDouble())
    val Number.rad get() = this.toDouble()

    @DslMarker
    private annotation class PlayerConfigurationDsl

    @PlayerConfigurationDsl
    class PlayerConfiguration : Configurable<PlayerConfiguration>() {
        companion object {
            fun of(configure: PlayerConfiguration.() -> Unit): PlayerConfiguration {
                val pc = PlayerConfiguration()
                pc.configure()
                return pc
            }
        }

        val Number.deg get() = toRadians(this.toDouble())
        val Number.rad get() = this.toDouble()

        @PlayerConfigurationDsl
        class LenienceRules(var radius: Double, var angleDelta: Double, var maxVel: Double) : Configurable<LenienceRules>() {
            val Number.deg get() = toRadians(this.toDouble())
            val Number.rad get() = this.toDouble()
        }

        val blocking = LenienceRules(0.5, 5.deg, 6.0)
        val nonBlocking = LenienceRules(2.0, 15.deg, -1.0)
    }

    operator fun invoke(configure: Player.() -> Unit): Player {
        this.configure()
        return this
    }

    // Intentionally immutable.
    val commands: List<BytecodeUnit>

    // Task-y things
    override var name = "[Player of ${filepath.name}]"
        set(_) { /* reject */ }
    override val daemon = false

    var done = false

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
        val angDist = currentPose.subtractAngle(targetPose)
        val targets = if (isNextBlocking()) options.blocking else options.nonBlocking

    }

    fun checkAwait(awaitCommand: AwaitCommand): Boolean {
        TODO()
    }

    fun isNextBlocking(): Boolean {
        // we need to grab a local copy so that the compiler can
        // assert that the value cannot change between check and use
        val nextCommand = nextCommand
        return when (nextCommand) {
            is AwaitCommand -> TODO()
            is MoveCommand -> false
            is RunAsyncCommand -> false
            is RunCommand -> true
            null -> true
        }
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