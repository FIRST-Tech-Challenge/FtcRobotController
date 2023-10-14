package org.firstinspires.ftc.teamcode.team10515.control;

import com.qualcomm.robotcore.hardware.Gamepad;

public class EnhancedGamepad {
    private Gamepad gamepad;

    private boolean dpadUpLast;
    private boolean dpadDownLast;
    private boolean dpadLeftLast;
    private boolean dpadRightLast;
    private boolean aLast;
    private boolean bLast;
    private boolean xLast;
    private boolean yLast;
    private boolean leftBumperLast;
    private boolean rightBumperLast;
    private boolean leftStickButtonLast;
    private boolean rightStickButtonLast;

    private boolean dpadUpJustPressed;
    private boolean dpadDownJustPressed;
    private boolean dpadLeftJustPressed;
    private boolean dpadRightJustPressed;
    private boolean aJustPressed;
    private boolean bJustPressed;
    private boolean xJustPressed;
    private boolean yJustPressed;
    private boolean leftBumperJustPressed;
    private boolean rightBumperJustPressed;
    private boolean leftStickButtonJustPressed;
    private boolean rightStickButtonJustPressed;

    private boolean dpadUpJustReleased;
    private boolean dpadDownJustReleased;
    private boolean dpadLeftJustReleased;
    private boolean dpadRightJustReleased;
    private boolean aJustReleased;
    private boolean bJustReleased;
    private boolean xJustReleased;
    private boolean yJustReleased;
    private boolean leftBumperJustReleased;
    private boolean rightBumperJustReleased;
    private boolean leftStickButtonJustReleased;
    private boolean rightStickButtonJustReleased;

    public EnhancedGamepad(Gamepad gamepad) {
        setGamepad(gamepad);
        setDpadUpJustPressed(false);
        setDpadDownJustPressed(false);
        setDpadLeftJustPressed(false);
        setDpadRightJustPressed(false);
        setaJustPressed(false);
        setbJustPressed(false);
        setxJustPressed(false);
        setyJustPressed(false);
        setLeftBumperJustPressed(false);
        setRightBumperJustPressed(false);
        setLeftStickButtonJustPressed(false);
        setRightStickButtonJustPressed(false);
    }

    public void update() {
        if(getGamepad() != null) {
            setDpadUpJustPressed(getGamepad().dpad_up && !isDpadUpLast());
            setDpadDownJustPressed(getGamepad().dpad_down && !isDpadDownLast());
            setDpadLeftJustPressed(getGamepad().dpad_left && !isDpadLeftLast());
            setDpadRightJustPressed(getGamepad().dpad_right && !isDpadRightLast());
            setaJustPressed(getGamepad().a && !isaLast());
            setbJustPressed(getGamepad().b && !isbLast());
            setxJustPressed(getGamepad().x && !isxLast());
            setyJustPressed(getGamepad().y && !isyLast());
            setLeftBumperJustPressed(getGamepad().left_bumper && !isLeftBumperLast());
            setRightBumperJustPressed(getGamepad().right_bumper && !isRightBumperLast());
            setLeftStickButtonJustPressed(getGamepad().left_stick_button && !isLeftStickButtonLast());
            setRightStickButtonJustPressed(getGamepad().right_stick_button && !isRightStickButtonLast());

            setDpadDownJustReleased(!getGamepad().dpad_up && isDpadDownLast());
            setDpadUpJustReleased(!getGamepad().dpad_down && isDpadUpLast());
            setDpadLeftJustReleased(!getGamepad().dpad_left && isDpadLeftLast());
            setDpadRightJustReleased(!getGamepad().dpad_right && isDpadRightLast());
            setaJustReleased(!getGamepad().a && isaLast());
            setbJustReleased(!getGamepad().b && isbLast());
            setxJustReleased(!getGamepad().x && isxLast());
            setyJustReleased(!getGamepad().y && isyLast());
            setLeftBumperJustReleased(!getGamepad().left_bumper && isLeftBumperLast());
            setRightBumperJustReleased(!getGamepad().right_bumper && isRightBumperLast());
            setLeftStickButtonJustReleased(!getGamepad().left_stick_button && isLeftStickButtonLast());
            setRightStickButtonJustReleased(!getGamepad().right_stick_button && isRightStickButtonLast());

            setDpadUpLast(getGamepad().dpad_up);
            setDpadDownLast(getGamepad().dpad_down);
            setDpadLeftLast(getGamepad().dpad_left);
            setDpadRightLast(getGamepad().dpad_right);
            setaLast(getGamepad().a);
            setbLast(getGamepad().b);
            setxLast(getGamepad().x);
            setyLast(getGamepad().y);
            setLeftBumperLast(getGamepad().left_bumper);
            setRightBumperLast(getGamepad().right_bumper);
            setLeftStickButtonLast(getGamepad().left_stick_button);
            setRightStickButtonLast(getGamepad().right_stick_button);
        }
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    private void setGamepad(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public boolean isDpadUpJustPressed() {
        return dpadUpJustPressed;
    }

    private void setDpadUpJustPressed(boolean dpadUpJustPressed) {
        this.dpadUpJustPressed = dpadUpJustPressed;
    }

    public boolean isDpadDownJustPressed() {
        return dpadDownJustPressed;
    }

    private void setDpadDownJustPressed(boolean dpadDownJustPressed) {
        this.dpadDownJustPressed = dpadDownJustPressed;
    }

    public boolean isDpadLeftJustPressed() {
        return dpadLeftJustPressed;
    }

    private void setDpadLeftJustPressed(boolean dpadLeftJustPressed) {
        this.dpadLeftJustPressed = dpadLeftJustPressed;
    }

    public boolean isDpadRightJustPressed() {
        return dpadRightJustPressed;
    }

    private void setDpadRightJustPressed(boolean dpadRightJustPressed) {
        this.dpadRightJustPressed = dpadRightJustPressed;
    }

    public boolean isaJustPressed() {
        return aJustPressed;
    }

    private void setaJustPressed(boolean aJustPressed) {
        this.aJustPressed = aJustPressed;
    }

    public boolean isbJustPressed() {
        return bJustPressed;
    }

    private void setbJustPressed(boolean bJustPressed) {
        this.bJustPressed = bJustPressed;
    }

    public boolean isxJustPressed() {
        return xJustPressed;
    }

    private void setxJustPressed(boolean xJustPressed) {
        this.xJustPressed = xJustPressed;
    }

    public boolean isyJustPressed() {
        return yJustPressed;
    }

    private void setyJustPressed(boolean yJustPressed) {
        this.yJustPressed = yJustPressed;
    }

    public boolean isLeftBumperJustPressed() {
        return leftBumperJustPressed;
    }

    private void setLeftBumperJustPressed(boolean leftBumperJustPressed) {
        this.leftBumperJustPressed = leftBumperJustPressed;
    }

    public boolean isRightBumperJustPressed() {
        return rightBumperJustPressed;
    }

    private void setRightBumperJustPressed(boolean rightBumperJustPressed) {
        this.rightBumperJustPressed = rightBumperJustPressed;
    }

    public boolean isLeftStickButtonJustPressed() {
        return leftStickButtonJustPressed;
    }

    private void setLeftStickButtonJustPressed(boolean leftStickButtonJustPressed) {
        this.leftStickButtonJustPressed = leftStickButtonJustPressed;
    }

    public boolean isRightStickButtonJustPressed() {
        return rightStickButtonJustPressed;
    }

    private void setRightStickButtonJustPressed(boolean rightStickButtonJustPressed) {
        this.rightStickButtonJustPressed = rightStickButtonJustPressed;
    }

    public float getLeft_stick_x() {
        return getGamepad().left_stick_x;
    }

    public float getLeft_stick_y() {
        return getGamepad().left_stick_y;
    }

    public float getRight_stick_x() {
        return getGamepad().right_stick_x;
    }

    public float getRight_stick_y() {
        return getGamepad().right_stick_y;
    }

    public boolean isDpad_up() {
        return getGamepad().dpad_up;
    }

    public boolean isDpad_down() {
        return getGamepad().dpad_down;
    }

    public boolean isDpad_left() {
        return getGamepad().dpad_left;
    }

    public boolean isDpad_right() {
        return getGamepad().dpad_right;
    }

    public boolean isA() {
        return getGamepad().a;
    }

    public boolean isB() {
        return getGamepad().b;
    }

    public boolean isX() {
        return getGamepad().x;
    }

    public boolean isY() {
        return getGamepad().y;
    }

    public boolean isGuide() {
        return getGamepad().guide;
    }

    public boolean isStart() {
        return getGamepad().start;
    }

    public boolean isBack() {
        return getGamepad().back;
    }

    public boolean isLeft_bumper() {
        return getGamepad().left_bumper;
    }

    public boolean isRight_bumper() {
        return getGamepad().right_bumper;
    }

    public boolean isLeft_stick_button() {
        return getGamepad().left_stick_button;
    }

    public boolean isRight_stick_button() {
        return getGamepad().right_stick_button;
    }

    public float getLeft_trigger() {
        return getGamepad().left_trigger;
    }

    public float getRight_trigger() {
        return getGamepad().right_trigger;
    }

    public boolean isDpadUpJustReleased() {
        return dpadUpJustReleased;
    }

    public void setDpadUpJustReleased(boolean dpadUpJustReleased) {
        this.dpadUpJustReleased = dpadUpJustReleased;
    }

    public boolean isDpadDownJustReleased() {
        return dpadDownJustReleased;
    }

    public void setDpadDownJustReleased(boolean dpadDownJustReleased) {
        this.dpadDownJustReleased = dpadDownJustReleased;
    }

    public boolean isDpadLeftJustReleased() {
        return dpadLeftJustReleased;
    }

    public void setDpadLeftJustReleased(boolean dpadLeftJustReleased) {
        this.dpadLeftJustReleased = dpadLeftJustReleased;
    }

    public boolean isDpadRightJustReleased() {
        return dpadRightJustReleased;
    }

    public void setDpadRightJustReleased(boolean dpadRightJustReleased) {
        this.dpadRightJustReleased = dpadRightJustReleased;
    }

    public boolean isaJustReleased() {
        return aJustReleased;
    }

    public void setaJustReleased(boolean aJustReleased) {
        this.aJustReleased = aJustReleased;
    }

    public boolean isbJustReleased() {
        return bJustReleased;
    }

    public void setbJustReleased(boolean bJustReleased) {
        this.bJustReleased = bJustReleased;
    }

    public boolean isxJustReleased() {
        return xJustReleased;
    }

    public void setxJustReleased(boolean xJustReleased) {
        this.xJustReleased = xJustReleased;
    }

    public boolean isyJustReleased() {
        return yJustReleased;
    }

    public void setyJustReleased(boolean yJustReleased) {
        this.yJustReleased = yJustReleased;
    }

    public boolean isLeftBumperJustReleased() {
        return leftBumperJustReleased;
    }

    public void setLeftBumperJustReleased(boolean leftBumperJustReleased) {
        this.leftBumperJustReleased = leftBumperJustReleased;
    }

    public boolean isRightBumperJustReleased() {
        return rightBumperJustReleased;
    }

    public void setRightBumperJustReleased(boolean rightBumperJustReleased) {
        this.rightBumperJustReleased = rightBumperJustReleased;
    }

    public boolean isLeftStickButtonJustReleased() {
        return leftStickButtonJustReleased;
    }

    public void setLeftStickButtonJustReleased(boolean leftStickButtonJustReleased) {
        this.leftStickButtonJustReleased = leftStickButtonJustReleased;
    }

    public boolean isRightStickButtonJustReleased() {
        return rightStickButtonJustReleased;
    }

    public void setRightStickButtonJustReleased(boolean rightStickButtonJustReleased) {
        this.rightStickButtonJustReleased = rightStickButtonJustReleased;
    }

    public boolean isDpadUpLast() {
        return dpadUpLast;
    }

    public void setDpadUpLast(boolean dpadUpLast) {
        this.dpadUpLast = dpadUpLast;
    }

    public boolean isDpadDownLast() {
        return dpadDownLast;
    }

    public void setDpadDownLast(boolean dpadDownLast) {
        this.dpadDownLast = dpadDownLast;
    }

    public boolean isDpadLeftLast() {
        return dpadLeftLast;
    }

    public void setDpadLeftLast(boolean dpadLeftLast) {
        this.dpadLeftLast = dpadLeftLast;
    }

    public boolean isDpadRightLast() {
        return dpadRightLast;
    }

    public void setDpadRightLast(boolean dpadRightLast) {
        this.dpadRightLast = dpadRightLast;
    }

    public boolean isaLast() {
        return aLast;
    }

    public void setaLast(boolean aLast) {
        this.aLast = aLast;
    }

    public boolean isbLast() {
        return bLast;
    }

    public void setbLast(boolean bLast) {
        this.bLast = bLast;
    }

    public boolean isxLast() {
        return xLast;
    }

    public void setxLast(boolean xLast) {
        this.xLast = xLast;
    }

    public boolean isyLast() {
        return yLast;
    }

    public void setyLast(boolean yLast) {
        this.yLast = yLast;
    }

    public boolean isLeftBumperLast() {
        return leftBumperLast;
    }

    public void setLeftBumperLast(boolean leftBumperLast) {
        this.leftBumperLast = leftBumperLast;
    }

    public boolean isRightBumperLast() {
        return rightBumperLast;
    }

    public void setRightBumperLast(boolean rightBumperLast) {
        this.rightBumperLast = rightBumperLast;
    }

    public boolean isLeftStickButtonLast() {
        return leftStickButtonLast;
    }

    public void setLeftStickButtonLast(boolean leftStickButtonLast) {
        this.leftStickButtonLast = leftStickButtonLast;
    }

    public boolean isRightStickButtonLast() {
        return rightStickButtonLast;
    }

    public void setRightStickButtonLast(boolean rightStickButtonLast) {
        this.rightStickButtonLast = rightStickButtonLast;
    }
}
