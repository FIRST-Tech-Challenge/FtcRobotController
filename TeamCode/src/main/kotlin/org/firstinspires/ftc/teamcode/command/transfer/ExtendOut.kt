package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class ExtendOut(private val viperArmSubsystem: ViperArmSubsystem) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = ExtendOutState.IDLE

    override fun initialize() {
        currentState = ExtendOutState.STARTED
    }

    override fun execute() {
        when (currentState) {
            ExtendOutState.STARTED -> {
                viperArmSubsystem.setExtensionMotorGroupPower(0.5)
                currentState = ExtendOutState.EXTENDING
            }
            ExtendOutState.EXTENDING -> {
                // TODO Adjust target rotation position
                if (viperArmSubsystem.getExtensionMotorGroupSpeed() >= 1000.0) {
                    viperArmSubsystem.setExtensionMotorGroupPower(0.0)
                    currentState = ExtendOutState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == ExtendOutState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            viperArmSubsystem.setExtensionMotorGroupPower(0.0)
        }

        currentState = ExtendOutState.IDLE
    }

    companion object {
        enum class ExtendOutState {
            IDLE,
            STARTED,
            EXTENDING,
            FINISHED
        }
    }
}