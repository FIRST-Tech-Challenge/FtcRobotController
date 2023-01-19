package teamcode.v1.commands.sequences

import com.asiankoala.koawalib.command.commands.InstantCmd
import com.asiankoala.koawalib.command.commands.WaitCmd
import com.asiankoala.koawalib.command.commands.WaitUntilCmd
import com.asiankoala.koawalib.command.group.ParallelGroup
import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.ClawCmds
import org.firstinspires.ftc.teamcode.koawalib.commands.subsystems.LiftCmds
import teamcode.v1.subsystems.Arm
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Claw
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Lift
import teamcode.v1.constants.ClawConstants

class HomeSequence(
    lift: Lift,
    claw : Claw,
    arm : Arm,
    firstArmAngle : Double,
    secondArmAngle : Double,
    clawPos : Double? = null
) : ParallelGroup(
    InstantCmd({
        if (clawPos != null) {
            ClawConstants.openPos = clawPos
        }
    }
    ),
    SequentialGroup(
        InstantCmd({arm.setPos(firstArmAngle)}, arm),
        WaitCmd(1.0),
        ClawCmds.ClawOpenCmd(claw),
        InstantCmd({arm.setPos(secondArmAngle)}, arm)),
    LiftCmds.LiftHomeCmd(lift),
    ClawCmds.ClawCloseCmd(claw),
    )

