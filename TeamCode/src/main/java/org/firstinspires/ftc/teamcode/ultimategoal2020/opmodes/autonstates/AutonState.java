package org.firstinspires.ftc.teamcode.ultimategoal2020.opmodes.autonstates;

public interface AutonState {
    /**
     * AutonState is an interface defining common methods for autonomous states
     * It is used along with AutonStateFactory to generate instances of AutonStates
     * The enumeration AutonStateEnum controls the available options
     */

    public Class<? extends AbstractAutonState> getNextAutonState();

    public Class<? extends AbstractAutonState> getCurrentAutonState();

    public boolean areExitConditionsMet();

    public void performStateSpecificTransitionActions();

    public void performStateActions();
}
