package org.firstinspires.ftc.teamcode.koawalib.commands

import com.asiankoala.koawalib.command.group.ParallelGroup
import org.firstinspires.ftc.teamcode.koawalib.Robot
import org.firstinspires.ftc.teamcode.koawalib.RobotState
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ArmCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift

object CmdChooser {
    fun liftCmd(lift: Lift): LiftCmds.LiftCmd {
        return when (RobotState.strategy) {
            RobotState.DepositState.HIGH -> LiftCmds.LiftHighCmd(lift)
            RobotState.DepositState.MEDIUM -> LiftCmds.LiftMidCmd(lift)
            RobotState.DepositState.LOW -> LiftCmds.LiftLowCmd(lift)
            RobotState.DepositState.GROUND -> LiftCmds.LiftGroundCmd(lift)
        }
    }

    fun armCmd(arm: Arm): ArmCmds.ArmCmd {
        return when (RobotState.strategy) {
            RobotState.DepositState.HIGH -> ArmCmds.ArmHighCmd(arm)
            RobotState.DepositState.MEDIUM -> ArmCmds.ArmMidCmd(arm)
            RobotState.DepositState.LOW -> ArmCmds.ArmLowCmd(arm)
            RobotState.DepositState.GROUND -> ArmCmds.ArmGroundCmd(arm)
        }
    }

    fun homeCmd(robot: Robot): ParallelGroup {
        return ParallelGroup(
            LiftCmds.LiftHomeCmd(robot.lift),
            ArmCmds.ArmHomeCmd(robot.arm),
            ClawCmds.ClawOpenCmd(robot.claw)
        )
    }

    fun double(robot: Robot): ParallelGroup {
        return ParallelGroup(
            liftCmd(robot.lift),
            armCmd(robot.arm),
        )
    }
}
