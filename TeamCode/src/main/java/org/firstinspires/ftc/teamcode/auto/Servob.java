package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;

public class Servob
{
    public final String servoName;
    private Servo servo;
    private final double openPosition;
    private final double closePosition;
    private final double increaseAmount;
    public Servob(String servoName, double openPosition, double closePosition, double increaseAmount){
        this.increaseAmount = increaseAmount==0.0 ? 0.05 : increaseAmount;
        this.servoName = servoName;
        this.openPosition = openPosition;
        this.closePosition = closePosition;
    }

    public void setupServo(){
        close();
    }
    public void setServo(Servo servo){
        this.servo = servo;
    }

    public void open(){
        servo.setPosition(openPosition);
    }
    public void close(){
        servo.setPosition(closePosition);
    }


    public void increase(){
        servo.setPosition(servo.getPosition()+increaseAmount);
    }
    public void decrease(){
        servo.setPosition(servo.getPosition()-increaseAmount);
    }

    public Servo getServo(){
        return servo;
    }
}
