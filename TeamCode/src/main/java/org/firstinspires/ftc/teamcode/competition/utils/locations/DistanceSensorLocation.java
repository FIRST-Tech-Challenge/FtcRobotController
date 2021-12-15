package org.firstinspires.ftc.teamcode.competition.utils.locations;

import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;

public class DistanceSensorLocation extends Location {



    @Override
    public void stop() {

    }

    @Override
    public boolean isInputLocation() {
        return true;
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
