package org.firstinspires.ftc.teamcode.playmaker;

@Deprecated
public class GamepadListener {

    enum GamepadListenerType {
        HOLD_RELEASE,
        PRESS,
        TOGGLE,
        AUTO_TRIGGER
    }

    public GamepadController.GamepadType type;
    public GamepadController.GamepadButtons button;
    private GamepadListenerType listenerType;
    public GamepadInterface activateInterface;
    public GamepadInterface deactivateInterface;

    private GamepadListener(GamepadListenerType type, GamepadController.GamepadType gamepadType, GamepadController.GamepadButtons button) {
        this.listenerType = type;
        this.type = gamepadType;
        this.button = button;
    }

    public static GamepadListener createHoldAndReleaseListener(GamepadController.GamepadType gamepadType, GamepadController.GamepadButtons button, GamepadInterface activateInterface, GamepadInterface deactivateInterface) {
        GamepadListener listener = new GamepadListener(GamepadListenerType.HOLD_RELEASE, gamepadType, button);
        listener.activateInterface = activateInterface;
        listener.deactivateInterface = deactivateInterface;
        return listener;
    }

    public static GamepadListener createPressListener(GamepadController.GamepadType gamepadType, GamepadController.GamepadButtons button, GamepadInterface gamepadInterface) {
        GamepadListener listener = new GamepadListener(GamepadListenerType.PRESS, gamepadType, button);
        listener.activateInterface = gamepadInterface;
        return listener;
    }

    public static GamepadListener createToggleListener(GamepadController.GamepadType gamepadType, GamepadController.GamepadButtons button, GamepadInterface activateInterface, GamepadInterface deactivateInterface) {
        GamepadListener listener = new GamepadListener(GamepadListenerType.TOGGLE, gamepadType, button);
        listener.activateInterface = activateInterface;
        listener.deactivateInterface = deactivateInterface;
        return listener;
    }

    public static GamepadListener createAutoTrigger(GamepadController.GamepadType gamepadType, GamepadController.GamepadButtons button, HybridOpController hybridOp, ActionSequence sequence, boolean haltManual) {
        /*GamepadListener listener = new GamepadListener(GamepadListenerType.AUTO_TRIGGER, gamepadType, button);
        listener.activateInterface = () -> {
            if (hybridOp.isAutonomous()) {
                hybridOp.stopAutonomous();
            } else {
                hybridOp.executeActionSequence(sequence, haltManual);
            }
        };*/
        return null;
    }

    public GamepadListenerType getListenerType() {
        return listenerType;
    }

}
