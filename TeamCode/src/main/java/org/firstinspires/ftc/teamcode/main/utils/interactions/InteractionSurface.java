package org.firstinspires.ftc.teamcode.main.utils.interactions;

/**
 * An InteractionSurface represents a Surface which can be interacted with. The item can either take input in, send output out, or both. The Surface is either an InteractionItem or InteractionGroup. InteractionItems represent either a single device or set of devices which perform a specific action. InteractionGroups represent a group of InteractionItems. InteractionItems can control their device(s), and InteractionGroups can control their InteractionItems.
 */
public abstract class InteractionSurface {

    public abstract boolean isInputDevice();

    public abstract boolean isOutputDevice();

}
