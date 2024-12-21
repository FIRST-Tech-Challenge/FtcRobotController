package org.firstinspires.ftc.teamcode.auto.servo;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.auto.servo.Servob;


// a adjustable servo class ith a amount to increase by
public class AjustableServo extends Servob {
    private final double StartFrom;
    private final double EndAt;
    private final double increaseAmount;
    private final double startPos;

    public AjustableServo(String servoName, double startPos, double StartFrom, double EndAt, double increaseAmount){
        super(servoName);
        this.increaseAmount = increaseAmount==0.0 ? 0.05 : increaseAmount;
        this.StartFrom = StartFrom;
        this.EndAt = EndAt;
        this.startPos = startPos;
    }


    public void startServo(){
        servo.setPosition(startPos);
    }

    public double checkAdjust(double value) {
        // Check if the value is within the range
        if (value >= StartFrom && value <= EndAt) {
            return value;
        } else {
            // Return the closest boundary (startFrom or endAt)
            if (Math.abs(value - StartFrom) < Math.abs(value - EndAt)) {
                return StartFrom;
            } else {
                return EndAt;
            }
        }
    }
    public void increase(){
        servo.setPosition(checkAdjust(servo.getPosition()+increaseAmount));
    }
    public void decrease(){
        servo.setPosition(checkAdjust(servo.getPosition()-increaseAmount));
    }

    public void set(double value){
        servo.setPosition(value);
    }
}
