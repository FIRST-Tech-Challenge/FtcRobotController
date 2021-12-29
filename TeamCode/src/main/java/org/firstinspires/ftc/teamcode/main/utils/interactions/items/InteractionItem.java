package org.firstinspires.ftc.teamcode.main.utils.interactions.items;

import org.firstinspires.ftc.teamcode.main.utils.interactions.InteractionSurface;

/**
 * InteractionItems represent either a single device or set of devices which can be controlled to perform a specific action.
 */
public abstract class InteractionItem extends InteractionSurface {

    public boolean isInteractionItem() {
        return true;
    }

    public boolean isInteractionGroup() {
        return false;
    }

}
