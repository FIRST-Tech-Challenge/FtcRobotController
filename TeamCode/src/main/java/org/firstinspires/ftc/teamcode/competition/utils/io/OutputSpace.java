package org.firstinspires.ftc.teamcode.competition.utils.io;

/**
 * This class can be used to receive output from locations. Locations get data from the interaction surfaces they handle, and then return that to this space. For example, if the location of touch sensor $y finds out $y is being touched, the location will send that data to the space on request. If something fails, it will consume any exceptions and act like nothing happened, returning an arbitrary value. When creating an OutputSpace, the OutputSpace attempts to create locations for every output location on the robot. For example, if the team decides to build a robot with 4 motors, locations for all 4 will be created and able to receive data from. The possible locations are defined inside this class. Since the OutputSpace is designed to handle output from the current robot, it should be built with the current robot and only the current robot in mind. Location classes are where all used locations should go. If they're unused on the current robot, leave them there. They should never be removed because they could be used later, and it's not like they really take up any extra resources anyway.
 */
public class OutputSpace {

    public OutputSpace() {

    }

}
