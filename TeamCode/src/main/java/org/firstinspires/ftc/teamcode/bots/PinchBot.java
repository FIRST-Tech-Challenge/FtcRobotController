package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Timer;
import java.util.TimerTask;

public class PinchBot extends PivotBot{

    @Deprecated
    private boolean isOpen = false;
    @Deprecated
    private boolean specimenReady = false;

    public Servo pinch;

    public Servo rotate;

    private double rotateServoPos; //0.66 - 0.3

    public final double rotateServoInitialPos = 0.57;
    public final double rotateHorizontalLeftPos = 0.57;
    public final double rotateHorizontalRightPos = 0.24;
    public final double rotateVerticalPos = 0.43;

    public final double rotateMaxPos = 0.73;
    public final double rotateMinPos = 0.13;

    public final double pinchOpenPos = 0.82;
    public final double pinchClosePos = 0.65;
    public final double pinchWideOpenPos = 0.91;
    private Timer pinchTimer = new Timer();
    private TimerTask pinchTimerTask;
    public PinchBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        super.init(hardwareMap);
        pinch = hardwareMap.get(Servo.class, "pinch");
        rotate = hardwareMap.get(Servo.class, "rotate");

        closePinch();
        rotateToPos(rotateServoInitialPos);
    }

    protected void onTick() {
        super.onTick();

        rotate.setPosition(rotateServoPos);
    }

    public void openPinch(){
        isOpen = true;
        pinch.setPosition(pinchOpenPos);
    }

    public void closePinch(){
        isOpen = false;
        pinch.setPosition(pinchClosePos);
    }
    public void openPinchWide(){
        pinch.setPosition(pinchWideOpenPos);
    }

    /**
     * Cancels the pinch timer if it is running.
     */
    public void cancelPinchTimer(){
        pinchTimer.cancel();
    }

    @Deprecated
    public void autoPinch(){
        if(isOpen){
            closePinch();
        }
        if(!isOpen){
            openPinch();
        }
    }
    public void pinchControl(boolean open, boolean close){

        if (open) {
            openPinch();
        }
        if (close) {
            closePinch();
        }
    }

    public void rotateToPos(double pos){
        if (pos > rotateMaxPos) {
            pos = rotateMaxPos;
        }
        if (pos < rotateMinPos) {
            pos = rotateMinPos;
        }
        rotateServoPos = pos;
        rotate.setPosition(rotateServoPos);
    }
    public void rotateToInitialPos(){
        rotateToPos(rotateServoInitialPos);
    }
    public void rotateToVerticalPos(){
        rotateToPos(rotateVerticalPos);
    }

    /**
     * Rotates the pinch servo to a specified angle (in degrees).
     * The angle must be between -90 to 90 degrees.
     * @param angle
     */
    public void rotateToAngle(int angle){ //5216 - 4706
        assert angle > -165 && angle <= 165 : "Angle must be between -165 and 165 degrees";
        double relativeAngle = (-angle + 90) / 180.0;
        double pos = relativeAngle * (rotateHorizontalLeftPos - rotateHorizontalRightPos) + rotateHorizontalRightPos;
        rotateToPos(pos);
    }

    public void rotateControl(boolean left, boolean right){

        double pos = rotateServoPos;
        if(left){
            pos -= 0.01;
        }
        if(right){
            pos += 0.01;
        }
        rotateToPos(pos);

    }
}
