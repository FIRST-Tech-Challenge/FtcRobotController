package org.firstinspires.ftc.teamcode.Usefuls.Motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Usefuls.Math.M;


public class  ServoMotorBetter implements Motor {
    private Servo servo;
    private Servo.Direction direction;
    private double lowerBound = 0.0;
    private double upperBound = 1.0;
    private double position = 0.0;
    private double lastPosition = 0.0;

    public ServoMotorBetter(Servo dcMotorEx) {
        this.servo = dcMotorEx;
    }

    public ServoMotorBetter setLowerBound(double bound) {
        this.lowerBound = bound;
        return this;
    }
    public ServoMotorBetter setUpperBound(double bound) {
        this.upperBound = bound;
        return this;
    }

    public ServoMotorBetter setDirection(DcMotorEx.Direction direction) {
        switch (direction) {
            case FORWARD:
                this.direction = Servo.Direction.FORWARD;
                break;
            case REVERSE:
                this.direction = Servo.Direction.REVERSE;
                break;
        }
        return this;
    }

    public ServoMotorBetter setPosition(double position) {
        this.position = position;
        return this;
    }

    public double getPosition(){
        return this.position;
    }

    public ServoMotorBetter addPosition(double position) {
        this.position += position;
        return this;
    }
    public void resetEncoder() {
        //lmao its a servo you bozo
    }

    public void update() {
        if(Math.abs(this.position - this.lastPosition) > 0.0001) {
            this.servo.setPosition(M.lerp(this.lowerBound, this.upperBound, this.position));
        }
        this.lastPosition = this.position;
    }
}