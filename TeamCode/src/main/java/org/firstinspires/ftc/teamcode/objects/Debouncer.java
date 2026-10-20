package org.firstinspires.ftc.teamcode.objects;

/**
 * Turns a noisy button into one clean "pressed" event.
 *
 * <p>A button can flicker for a moment when pressed. The debouncer only says "pressed"
 * after it has seen the button held for several loops in a row. Once your code has acted
 * on the press, call {@link #Used()} so holding the button down doesn't act again every
 * loop. Letting go of the button gets it ready for the next press.
 *
 * <p>Each loop: call {@link #In} with the button, then check {@link #Out}.
 * <pre>
 *   mResetHeadingButton.In(gamepad.circle);
 *   if (mResetHeadingButton.Out()) {
 *       mResetHeadingButton.Used();
 *       // do the thing once
 *   }
 * </pre>
 */
public class Debouncer {
    /** How many loops in a row the button must be held before it counts as pressed. */
    private final int mLoopsNeeded;
    /** How many loops in a row the button has been held so far. */
    private int mLoopsHeld;
    /** true once this press has been acted on. Cleared when the button is let go. */
    private boolean mUsed;

    /**
     * @param loopsNeeded how many loops in a row the button must be held to count
     */
    public Debouncer(int loopsNeeded) {
        mLoopsNeeded = loopsNeeded;
        mLoopsHeld = 0;
        mUsed = false;
    }

    /**
     * Give the debouncer this loop's button reading.
     *
     * @param pressed true if the button is down right now
     */
    public void In(boolean pressed) {
        if (pressed) {
            // Count up, but stop at mLoopsNeeded so the number can't grow forever.
            if (mLoopsHeld < mLoopsNeeded) {
                mLoopsHeld++;
            }
        } else {
            // Button let go: start over, and get ready for the next press.
            mLoopsHeld = 0;
            mUsed = false;
        }
    }

    /**
     * @return true when the button has been held long enough and this press hasn't been used yet
     */
    public boolean Out() {
        boolean heldLongEnough = mLoopsHeld >= mLoopsNeeded;
        return heldLongEnough && !mUsed;
    }

    /** Mark this press as handled. {@link #Out()} stays false until the button is let go. */
    public void Used() {
        mUsed = true;
    }

    /** Act as if the button was just pressed and held long enough. */
    public void Force() {
        mLoopsHeld = mLoopsNeeded;
        mUsed = false;
    }
}
