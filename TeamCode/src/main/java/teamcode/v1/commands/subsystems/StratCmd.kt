package teamcode.v1.commands.subsystems

import com.asiankoala.koawalib.command.commands.InstantCmd
import teamcode.v1.Robot
import teamcode.v1.constants.Strategy

class StratCmd (robot: Robot, strategy: Strategy) : InstantCmd({robot.strat = strategy}){
}