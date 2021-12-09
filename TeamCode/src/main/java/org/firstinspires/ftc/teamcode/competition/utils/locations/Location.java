package org.firstinspires.ftc.teamcode.competition.utils.locations;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;

/**
 * Locations represent places which represent interaction surfaces that spaces can send and receive data to/from.
 * @implSpec Locations should attempt to take any valid input given and handle that to achieve the desired outcome from the interaction surface. If the undesired outcome occurs, the location should not throw an error, but may send data back via output methods. On that note, if the location receives data from the interaction surface, it should be able to send that data back. They should do these with the handleInput and returnOutput methods. These methods are not abstract, but are made in the subclass. The reason for this is so they can take different types of arguments. Locations should also be able to tell users whether they're an input location, output location, or both using implementations of the abstract getters with the correct purpose. Locations should be named following the name of the interaction surface it controls to the best of its ability in camel case. Locations should only handle one interaction surface unless necessary. If it makes sense for a location to control multiple surfaces, attempt to combine those interaction items into a interaction group first, though.
 */
public abstract class Location {

    public abstract boolean isInputLocation();

    public abstract boolean isOutputLocation();

    /**
     * This returns the internal interaction surface of the location, AKA what the location handles. For example, if this location was for motor $x, this would return the StandardMotor instance of motor $x.
     * @return The internal interaction surface of the location
     */
    public abstract InteractionSurface getInternalInteractionSurface();

}
