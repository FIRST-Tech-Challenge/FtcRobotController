package ftc.rogue.blacksmith.internal.scheduler

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.internal.util.DoubleConsumer
import ftc.rogue.blacksmith.internal.util.consume
import ftc.rogue.blacksmith.listeners.Listener

class SchedulerInternal {
    val listeners = mutableSetOf<Listener>()

    var beforeEach = Runnable {}

    fun launch(opmode: LinearOpMode, afterEach: Runnable) {
        Scheduler.emit(Scheduler.STARTING_MSG)

        while (opmode.opModeIsActive() && !opmode.isStopRequested) {
            updateListenersSet()

            beforeEach.run()
            tick()
            afterEach.run()
        }
    }

    fun launchOnStart(opmode: LinearOpMode, afterEach: Runnable) {
        opmode.waitForStart()
        launch(opmode, afterEach)
    }

    fun time(opmode: LinearOpMode, afterEach: DoubleConsumer) {
        Scheduler.emit(Scheduler.STARTING_MSG)

        val elapsedTime = ElapsedTime()

        while (opmode.opModeIsActive() && !opmode.isStopRequested) {
            updateListenersSet()

            beforeEach.run()
            tick()

            elapsedTime.milliseconds().let {
                afterEach.consume(it)
            }

            elapsedTime.reset()
        }
    }

    fun manuallyUpdateListeners() {
        updateListenersSet()
        tick()
    }

    fun reset() {
        listeners.clear()
        beforeEach = Runnable {}
    }

    private val messages = mutableMapOf<Any, MutableList<Runnable>>()

    fun on(message: Any, callback: Runnable) {
        messages.getOrPut(message, ::ArrayList) += callback
    }

    fun emit(message: Any) {
        messages[message]?.forEach(Runnable::run)
    }

    private val listenersToAdd = mutableSetOf<Listener>()
    private val listenersToRemove = mutableSetOf<Listener>()

    fun hookListener(listener: Listener) {
        listenersToAdd += listener
    }

    fun unhookListener(listener: Listener) {
        listenersToRemove += listener
    }

    private fun updateListenersSet() {
        listeners += listenersToAdd
        listenersToAdd.clear()
        listeners -= listenersToRemove
        listenersToRemove.clear()
    }

    private fun tick() = listeners.forEach(Listener::tick)
}
