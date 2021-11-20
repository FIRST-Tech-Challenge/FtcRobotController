package org.firstinspires.ftc.teamcode.competition.utils.teleop;

public class TeleOpSubsystems {

    private final boolean INTAKE, DUCK, LIFT, HAND;

    public TeleOpSubsystems(boolean intake, boolean duck, boolean lift, boolean hand) {
        INTAKE = intake;
        DUCK = duck;
        LIFT = lift;
        HAND = hand;
    }

    public boolean isIntakeActive() {
        return INTAKE;
    }

    public boolean isDuckActive() {
        return DUCK;
    }

    public boolean isLiftActive() {
        return LIFT;
    }

    public boolean isHandActive() {
        return HAND;
    }

}
