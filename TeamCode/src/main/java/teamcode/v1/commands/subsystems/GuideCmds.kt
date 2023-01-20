package teamcode.v1.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import teamcode.v1.constants.GuideConstants
import teamcode.v1.subsystems.Claw
import teamcode.v1.subsystems.Guide

class GuideCmds {
    open class GuideCmd(guide: Guide, pos: Double) : InstantCmd({ guide.setPos(pos) }, guide)

    class GuideHomeCmd(guide: Guide) : GuideCmd(guide, GuideConstants.homePos)
    class GuideDepositCmd(guide: Guide) : GuideCmd(guide, GuideConstants.depositPos)
}