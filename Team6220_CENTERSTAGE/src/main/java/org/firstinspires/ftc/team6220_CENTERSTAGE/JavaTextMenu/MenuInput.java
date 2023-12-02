package org.firstinspires.ftc.team6220_CENTERSTAGE.JavaTextMenu;

/**
 * A class for bridging input methods to a text menu.
 * <p>
 * This is made with the intention of supporting controllers!
 * That means that when passing input, one should set the input
 * type to {@code InputType.CONTROLLER} in the constructor. The user can
 * then pass raw input into the {@code .update(...)} method and it
 * will be accordingly processed.
 */
public class MenuInput {

    // determines the type of input processing
    // raw is good for standalone console debugging via keyboard
    private final InputType inputType;
    public enum InputType {
        CONTROLLER,
        RAW,
    }

    // a constant for stick deadzone
    public static final double INPUT_DEADZONE = 0.5;
    // constants for spacing out sustained input (in seconds)
    private static final double STICK_TAP_COOLDOWN = 0.3;
    private static final double STICK_HOLD_COOLDOWN = 0.1;

    // processed values
    private int x, y;
    // input timer to stop input spamming
    private double stickTimer; // in seconds
    // switches on when past tap cooldown, then using hold cooldown
    private boolean isHoldingStick = false;
    // time passed between update calls to update timers with
    private double deltaTime = 0.0;
    private Long lastTime = null;

    // processed value
    private boolean select;
    // so it only registers once per held press
    private boolean hasAlreadySelected = false;

    /**
     * creates a new input processing object.
     * @param inputType determines the type of input processing
     */
    public MenuInput(InputType inputType) {
        this.inputType = inputType;
    }
    /**
     * creates a new input processing object.
     * <b>defaults to raw input type.</b>
     */
    public MenuInput() {
        this(InputType.RAW);
    }

    /**
     * updates the input values. considers current input type.
     * @param x current raw x input
     * @param y current raw y input
     * @param select current raw select input
     * @return itself so you can use it immediately after updating it
     */
    public MenuInput update(double x, double y, boolean select) {

        // reset all
        this.select = false;
        this.x = 0;
        this.y = 0;

        switch (this.inputType) {

            case RAW:
            
                // directly apply the values
                this.select = select;
                this.x = (int)x;
                this.y = (int)y;

                break;

            case CONTROLLER:

                updateDeltaTime();

                // only register one select per sustained input:
                // initial select
                if (!this.hasAlreadySelected && select) {
                    this.select = true;
                    this.hasAlreadySelected = true;
                // if select button isn't held anymore reset it
                } else if (this.hasAlreadySelected && !select) {
                    this.hasAlreadySelected = false;
                }

                // get new x,y input; consider deadzone
                if (Math.hypot(x, y) > INPUT_DEADZONE) {
                    // snap input vector to axis
                    if (Math.abs(x) >= Math.abs(y)) {
                        this.x = (int)Math.signum(x);
                    } else {
                        this.y = (int)Math.signum(y);
                    }
                } else {
                    // stopped holding stick, reset timer and held status
                    this.isHoldingStick = false;
                    this.stickTimer = 0;
                }

                // slightly cursed stick input spacing implementation:
                // (sustained input -> repeated but spaced out input after an initial pause)

                // if it's the initial stick input:
                if (!this.isHoldingStick && this.stickTimer <= this.deltaTime) {
                    this.stickTimer += this.deltaTime;
                    // allows the x and y values to pass through
                } else {
                    this.stickTimer += this.deltaTime;

                    // if it's held for long enough to pass the tap cooldown
                    if (!this.isHoldingStick && this.stickTimer >= STICK_TAP_COOLDOWN) {
                        this.stickTimer = this.stickTimer % STICK_TAP_COOLDOWN;
                        this.isHoldingStick = true; // changes to holding cooldown mode
                        // allows the x and y values to pass through

                    // if it's past the tap cooldown and is also past the hold cooldown
                    } else if (this.isHoldingStick && this.stickTimer >= STICK_HOLD_COOLDOWN) {
                        this.stickTimer = this.stickTimer % STICK_HOLD_COOLDOWN;
                        // allows the x and y values to pass through

                    } else {
                        // otherwise block the x and y values
                        this.x = 0;
                        this.y = 0;
                    }
                }

                break;
        }
        
        return this;
    }

    // updates the deltatime in seconds
    private void updateDeltaTime() {
        if (this.lastTime != null) {
            this.deltaTime = (double)(System.nanoTime() - this.lastTime) / 1_000_000_000.0;
        }
        this.lastTime = System.nanoTime();
    } 

    public int getX() {
        return this.x;
    }
    public int getY() {
        return this.y;
    }
    public boolean getSelect() {
        return this.select;
    }   
}