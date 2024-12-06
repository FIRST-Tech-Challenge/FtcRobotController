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

    private double rotateServoPos;

    public final double rotateServoInitialPos = 0.80;
    public final double rotateServoMax = 0.80;
    public final double rotateServoMin = 0.46;
    public final double rotateVerticalPos = 0.63;

    public final double pinchOpenPos = 0.78;
    public final double pinchClosePos = 0.65;
    private Timer pinchTimer = new Timer();
    private TimerTask pinchTimerTask;
    public PinchBot(LinearOpMode opMode) {
        super(opMode);
    }


    public void init(HardwareMap hardwareMap){
        super.init(hardwareMap);
        pinch = hardwareMap.get(Servo.class, "pinch");
        rotate = hardwareMap.get(Servo.class, "rotate");

        openPinch();
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

    /**
     * Schedules the pinch servo to close after a specified time.
     * @param time The delay in milliseconds before closing the pinch.
     */
    public void closePinchInTime(int time){
        pinchTimerTask = new TimerTask() {
            @Override
            public void run() {
                closePinch();
            }
        };
        pinchTimer.schedule(pinchTimerTask, time);
    }

    /**
     * Schedules the pinch servo to open after a specified time.
     * @param time The delay in milliseconds before opening the pinch.
     */
    public void openPinchInTime(int time){
        pinchTimerTask = new TimerTask() {
            @Override
            public void run() {
                openPinch();
            }
        };
        pinchTimer.schedule(pinchTimerTask, time);
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
        if (pos > rotateServoMax) {
            pos = rotateServoMax;
        }
        if (pos < rotateServoMin) {
            pos = rotateServoMin;
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
        assert angle >= -90 && angle <= 90 : "Angle must be between -90 and 90 degrees";
        double relativeAngle = (-angle + 90) / 180.0;
        double pos = relativeAngle * (rotateServoMax - rotateServoMin) + rotateServoMin;
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
    @Deprecated
    public void pickUp(boolean button){
        // use horizontalDistance() to move robot and verticalDistance() to move slide
        // figure out how to find angle and use rotate(angle)

        double VERTICAL_OFFSET = 2;
        double VERTICAL_PROPORTION = 2;
        double HORIZONTAL_PROPORTION = 2;

        // TODO : refactor with new detectOne method
//        double[] position = detectOne();
//        double x = position[0];
//        double y = position[1];
//        double theta = position[2];
//        moveSlide((int) ((y + VERTICAL_OFFSET)*VERTICAL_PROPORTION),0.5); //move slide vertically
//        driveStraightByDistance(90, x*HORIZONTAL_PROPORTION, 2);
//        rotate(theta);
//        isOpen = false;
        //pinchControl();

        //slideControl(false, true);
        //rotate(CENTER_POSITION);

    }
}
