package org.firstinspires.ftc.teamcode.competition.utils.io;

/**
 * This class can be used to send input to locations. Locations then attempt to handle the input. For example, if the location of motor $x receives a value, the location will attempt to spin that motor at $x speed. If it fails, it will consume any exceptions and act like nothing happened. When creating an InputSpace, the InputSpace attempts to create locations for every input location on the robot. For example, if the team decides to build a robot with 4 motors, locations for all 4 will be created and able to send data to. The possible locations are defined inside this class. Since the InputSpace is designed to handle input for the current robot, it should be built with the current robot and only the current robot in mind. Location classes are where all used locations should go. If they're unused on the current robot, leave them there. They should never be removed because they could be used later, and it's not like they really take up any extra resources anyway.
 */
public class InputSpace {

    public InputSpace() {

    }

}
