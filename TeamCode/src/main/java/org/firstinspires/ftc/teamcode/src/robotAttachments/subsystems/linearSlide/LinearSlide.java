package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.src.utills.Controllable;

/**
 * A class to control linear slides using the builtin motor control methods
 */
public class LinearSlide implements Controllable<Void> {

    /**
     * The internal DcMotor object
     */
    final DcMotor linearSlide;
    private final ElapsedTime slideResetTimer = new ElapsedTime();
    private boolean resetSlide = false;
    private boolean manualSlideControl = false;
    private HeightLevel currentLevel = HeightLevel.Down;
    private boolean dPadUpDepressed = true;
    private boolean dPadDownDepressed = true;

    /**
     * A constructor for the linear slide
     *
     * @param hardwareMap The hardware map from the OpMode
     * @param slideName   The name of the slide
     */
    public LinearSlide(HardwareMap hardwareMap, String slideName) {
        linearSlide = hardwareMap.dcMotor.get(slideName);

        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(HeightLevel.getEncoderCountFromEnum(HeightLevel.Down));
        linearSlide.setPower(1);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Sets the linear slide to go to the provided position
     *
     * @param targetPosition The position to go to in ticks
     */
    public void setTargetPosition(int targetPosition) {
        linearSlide.setTargetPosition(targetPosition);
    }

    /**
     * Sets the slide to go to the provided level
     *
     * @param level The level to go to
     */
    public void setTargetLevel(HeightLevel level) {
        linearSlide.setTargetPosition(HeightLevel.getEncoderCountFromEnum(level));
    }

    /**
     * Draws on the internal REV positioning functions to determine if the slide is close
     *
     * @return If the slide is trying to get to a position
     */
    public boolean isAtPosition() {
        return !(linearSlide.isBusy());
    }

    /**
     * A getter for the position the slide is currently at
     *
     * @return The position of the slide in ticks
     */
    public int getEncoderCount() {
        return linearSlide.getCurrentPosition();
    }

    /**
     * Returns true if the slide is close to it's set position
     *
     * @param tolerance The maximum distance away the slide can be and still return true
     * @return true if the slide is close, false if it is not
     */
    public boolean isAtPosition(int tolerance) {
        return Math.abs(linearSlide.getCurrentPosition() - linearSlide.getTargetPosition()) < tolerance;
    }

    /**
     * Resets the encoders, reverts to autonomous mode
     */
    public void resetEncoder() {
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linearSlide.setTargetPosition(HeightLevel.getEncoderCountFromEnum(HeightLevel.Down));
        linearSlide.setPower(1);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /**
     * Changes the linear slide to work by setting power
     */
    public void teleopMode() {
        linearSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linearSlide.setPower(0);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Changes the linear slide to work by going to position
     */
    public void autoMode() {
        linearSlide.setTargetPosition(linearSlide.getCurrentPosition());
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(1);
    }

    /**
     * Reverses the motor
     */
    public void reverseMotor() {
        switch (linearSlide.getDirection()) {
            case FORWARD:
                linearSlide.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case REVERSE:
                linearSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    /**
     * Sets the power to the motor
     *
     * @param power The power to set to between the range of -1 and 1
     */
    public void setMotorPower(double power) {
        linearSlide.setPower(power);
    }

    /**
     * Getter for the target height
     *
     * @return Returns the position the Slide is set to go to
     */
    public int getTargetHeight() {
        return linearSlide.getTargetPosition();
    }

    /**
     * Allows control of a Linear Slide
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     * @return Always returns null
     */
    @Override
    public Void gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) {
        if (!resetSlide) {
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                manualSlideControl = true;
                int pos = (int) (Math.abs(this.getEncoderCount()) + (100 * -gamepad2.left_stick_y));
                if (pos < -100) pos = -100;
                if (pos > HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel))
                    pos = HeightLevel.getEncoderCountFromEnum(HeightLevel.TopLevel);
                this.setTargetPosition(pos);
            }


            if (!gamepad2.dpad_up) {
                dPadUpDepressed = true;
            }

            if (gamepad2.dpad_up && dPadUpDepressed) {
                if (manualSlideControl) {
                    manualSlideControl = false;
                    currentLevel = HeightLevel.getClosestLevel(this.getEncoderCount());
                }
                dPadUpDepressed = false;
                currentLevel = currentLevel.add(1);
                this.setTargetLevel(currentLevel);

            }

            if (!gamepad2.dpad_down) {
                dPadDownDepressed = true;
            }

            if (gamepad2.dpad_down && dPadDownDepressed) {
                if (manualSlideControl) {
                    manualSlideControl = false;
                    currentLevel = HeightLevel.getClosestLevel(this.getEncoderCount());
                }
                dPadDownDepressed = false;
                currentLevel = currentLevel.subtract(1);
                this.setTargetLevel(currentLevel);
            }

            if (gamepad2.right_bumper && gamepad2.left_bumper) {
                this.teleopMode();
                slideResetTimer.reset();
                resetSlide = true;
            }
        } else if (slideResetTimer.seconds() > 0.5 && slideResetTimer.seconds() < 0.6) {
            this.setMotorPower(0.3);
        } else if (slideResetTimer.seconds() > 1 && resetSlide) {
            resetSlide = false;
            this.autoMode();
            this.resetEncoder();
        }


        return null;
    }

    public void halt() {

    }
}
