package org.firstinspires.ftc.teamcode.competition.utils.locations;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;

/**
 * Locations represent places which represent interaction surfaces that spaces can send and receive data to/from.
 * @implSpec Locations should attempt to take any valid input given and handle that to achieve the desired outcome from the interaction surface. If there are multiple possible outcomes, the location should only take care of the most common outcomes and require the space using the location to interact with the internal surface directly for niche outcomes. If the undesired outcome occurs, the location should not throw an error, but may send data back via output methods. On that note, if the location receives data from the interaction surface, it should be able to send that data back, but only the most common data. Other data should be accessed directly from the surface, like secondary input. They should do these with the handleInput and returnOutput methods. The methods should define which type of input or output to handle or return with an enumerator taken as the first argument. If there's only one type of input or output, this argument is not required These methods are not abstract, but are made in the subclass. The reason for this is so they can take different types of arguments. Locations should also be able to tell users whether they're an input location, output location, or both using implementations of the abstract getters with the correct purpose. The names of the interaction surfaces locations handle should be easy to infer from the location, or written in an @apiNote. Lastly, locations should only handle one interaction surface. When multiple interaction surfaces make sense to be controlled by one location, think about making an grouped interaction surface.
 */
public abstract class Location {

    public abstract void stop();

    public abstract boolean isInputLocation();

    public abstract boolean isOutputLocation();

    /**
     * This returns the internal interaction surface of the location, AKA what the location handles. For example, if this location was for motor $x, this would return the StandardMotor instance of motor $x.
     * @return The internal interaction surface of the location
     */
    public abstract InteractionSurface getInternalInteractionSurface();

}
