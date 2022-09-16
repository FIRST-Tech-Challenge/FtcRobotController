package org.firstinspires.ftc.teamcode.stateMachine

import robotuprising.lib.system.statemachine.transition.TimedTransition
import robotuprising.lib.system.statemachine.transition.TransitionCondition

class StateMachineBuilder<StateEnum> {

    private val stateList = mutableListOf<State<StateEnum>>()

    private var exitToState: StateEnum? = null

    fun state(state: StateEnum): StateMachineBuilder<StateEnum> {
        if (stateList.find { it.state == state } != null)
            throw Error("State already exists in list")

        stateList.add(State(state, mutableListOf(), mutableListOf(), mutableListOf(), null))

        return this
    }

    fun transition(transitionCondition: TransitionCondition): StateMachineBuilder<StateEnum> {
        if (stateList.isEmpty())
            throw Error("No state to transition from")

        stateList.last().transitionCondition = transitionCondition
        return this
    }

    fun transitionTimed(time: Double): StateMachineBuilder<StateEnum> = transition(TimedTransition(time))

    fun onEnter(callback: Action): StateMachineBuilder<StateEnum> {
        if (stateList.isEmpty())
            throw Error("No state to modify")

        stateList.last().enterActions.add(callback)

        return this
    }

    fun onExit(callback: Action): StateMachineBuilder<StateEnum> {
        if (stateList.isEmpty())
            throw Error("No state to modify")

        stateList.last().exitActions.add(callback)

        return this
    }

    fun loop(callback: Action): StateMachineBuilder<StateEnum> {
        if (stateList.isEmpty())
            throw Error("No state to modify")

        stateList.last().loopActions.add(callback)

        return this
    }

    fun exit(exitToState: StateEnum): StateMachineBuilder<StateEnum> {
        this.exitToState = exitToState

        return this
    }

    fun build(): StateMachine<StateEnum> {
        return StateMachine(stateList, exitToState)
    }
}
