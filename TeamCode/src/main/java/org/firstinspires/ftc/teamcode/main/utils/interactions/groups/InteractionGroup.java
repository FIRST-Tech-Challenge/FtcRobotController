package org.firstinspires.ftc.teamcode.main.utils.interactions.groups;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

/**
 * InteractionGroups represent a controllable group of InteractionItems.
 */
public abstract class InteractionGroup extends InteractionSurface {

    public boolean isInteractionItem() {
        return false;
    }

    public boolean isInteractionGroup() {
        return true;
    }

}
