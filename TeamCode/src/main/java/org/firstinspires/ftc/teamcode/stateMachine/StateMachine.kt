package org.firstinspires.ftc.teamcode.stateMachine

import robotuprising.lib.system.statemachine.transition.TimedTransition

class StateMachine<StateEnum>(private val stateList: List<State<StateEnum>>, private val exitToState: StateEnum?) {
    var running = false
        private set

    private var currentState = stateList.first()

    fun start() {
        running = true

        if (currentState.transitionCondition is TimedTransition)
            (currentState.transitionCondition as TimedTransition).startTimer()

        currentState.enterActions.forEach { me -> me.run() }
    }

    fun stop() {
        running = false
    }

    fun reset() {
        currentState = stateList.first()
    }

    val state: StateEnum get() = currentState.state

    fun update() {
        if (currentState.transitionCondition?.shouldTransition() == true)
            transition()

        if (!running) return

        currentState.loopActions.forEach { it.run() }
    }

    // returns true if change in state
    // returns false if not
    private fun transition(): Boolean {
        currentState.exitActions.forEach { it.run() }

        if (stateList.last() == currentState) {
            running = false

            // If we want to exit to a certain state, set currentState to that state
            // Otherwise just return false and don't transition
            if (exitToState != null) {
                val searchForState = stateList.find { it.state == exitToState }

                if (searchForState != null) currentState = searchForState else return false
            } else {
                return false
            }
        } else {
            currentState = stateList[stateList.indexOf(currentState) + 1]
        }

        currentState.enterActions.forEach { it.run() }

        if (currentState.transitionCondition is TimedTransition)
            (currentState.transitionCondition as TimedTransition).startTimer()

        return true
    }

    fun smartRun(shouldStart: Boolean) {
        if (shouldStart) {
            reset()
            start()
        }

        if (running) {
            update()
        }
    }
}
