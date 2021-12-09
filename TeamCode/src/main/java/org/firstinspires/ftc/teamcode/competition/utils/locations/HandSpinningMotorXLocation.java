package org.firstinspires.ftc.teamcode.competition.utils.locations;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;

public class HandSpinningMotorXLocation extends Location {

    @Override
    public boolean isInputLocation() {
        return false;
    }

    @Override
    public boolean isOutputLocation() {
        return false;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return null;
    }

}
