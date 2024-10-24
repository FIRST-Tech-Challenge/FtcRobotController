package org.firstinspires.ftc.teamcode.command.transfer

import com.arcrobotics.ftclib.command.CommandBase
import org.firstinspires.ftc.teamcode.subsystem.ViperArmSubsystem

class RotateUp(private val viperArmSubsystem: ViperArmSubsystem) : CommandBase() {
    init {
        addRequirements(viperArmSubsystem)
    }

    private var currentState = RotateUpState.IDLE

    override fun initialize() {
        currentState = RotateUpState.STARTED
    }

    override fun execute() {
        when (currentState) {
            RotateUpState.STARTED -> {
                viperArmSubsystem.setRotationMotorGroupPower(0.5)
                currentState = RotateUpState.ROTATING
            }
            RotateUpState.ROTATING -> {
                // TODO Adjust target rotation position
                if (viperArmSubsystem.getRotationMotorGroupSpeed() >= 1000.0) {
                    viperArmSubsystem.setRotationMotorGroupPower(0.0)
                    currentState = RotateUpState.FINISHED
                }
            }
            else -> {
                // Do nothing
            }
        }
    }

    override fun isFinished(): Boolean {
        return currentState == RotateUpState.FINISHED
    }

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            viperArmSubsystem.setRotationMotorGroupPower(0.0)
        }

        currentState = RotateUpState.IDLE
    }

    companion object {
        enum class RotateUpState {
            IDLE,
            STARTED,
            ROTATING,
            FINISHED
        }
    }
}