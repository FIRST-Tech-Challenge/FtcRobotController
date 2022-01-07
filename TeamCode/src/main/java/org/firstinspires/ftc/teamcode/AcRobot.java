package org.firstinspires.ftc.teamcode;
import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.Constants;
import org.firstinspires.ftc.teamcode.math.Vector;

public class AcRobot {
    // movement
    public DcMotorEx leftFront = null;
    public DcMotorEx rightFront = null;
    public DcMotorEx leftRear = null;
    public DcMotorEx rightRear = null;

    // DcMotor
    public DcMotor carousel;

    public DigitalChannel grabberTouch = null;
    public DigitalChannel limitFront = null;
    public DigitalChannel limitRear = null;

    /* servos */
    public CRServo grabberRight = null;
    public CRServo grabberLeft = null;


    public AcRobot(){
    }

    //run this before anything else
    public void initHardware(HardwareMap hardwareMap){

        //initialize drive motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        //set two of the motors to be reversed
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        //Servos
        grabberLeft = hardwareMap.get(CRServo.class, "left");
        grabberRight = hardwareMap.get(CRServo.class, "right");

        //Sensors
        limitFront = hardwareMap.get(DigitalChannel.class, "armLimitFront");
        limitFront.setMode(DigitalChannel.Mode.INPUT);
        limitRear = hardwareMap.get(DigitalChannel.class, "armLimitRear");
        limitRear.setMode(DigitalChannel.Mode.INPUT);

        grabberTouch = hardwareMap.get(DigitalChannel.class, "grabberTouch");
        grabberTouch.setMode(DigitalChannel.Mode.INPUT);

        leftFront.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );
        rightFront.setMode( DcMotor.RunMode.RUN_USING_ENCODER );
        leftRear.setMode(   DcMotor.RunMode.RUN_USING_ENCODER );
        rightRear.setMode(  DcMotor.RunMode.RUN_USING_ENCODER );

    }

    public void DriveTo(Vector position, double duration){
        DriveWithVelocity(Vector.normalize(position), 0, (int)(position.magnitude()/duration));
        sleep((long)duration*1000);
        stop();
    }

    public void DriveWithVelocity(Vector direction, double rot, int speed){
        double TPR = Constants.motor5203TPR;
        double r = Math.hypot(direction.x, direction.y);
        double robotAngle = Math.atan2(direction.y, -direction.x) - Math.PI / 4;
        double rightX = -rot;
        final double lf = r * Math.cos(robotAngle) + rightX;
        final double rl = r * Math.sin(robotAngle) - rightX;
        final double lr = r * Math.sin(robotAngle) + rightX;
        final double rr = r * Math.cos(robotAngle) - rightX;

        leftFront.setVelocity(lf*((double)speed/360)*TPR);
        rightFront.setVelocity(rl*((double)speed/360)*TPR);
        leftRear.setVelocity(lr*((double)speed/360)*TPR);
        rightRear.setVelocity(rr*((double)speed/360)*TPR);
    }
    public void stop(){
        leftFront.setVelocity(0);
        rightFront.setVelocity(0);
        leftRear.setVelocity(0);
        rightRear.setVelocity(0);
    }

    // x and y are a vector direction
    public void DRIVE(double x, double y, double rot){
        double r = Math.hypot(x, y);
        double robotAngle = Math.atan2(y, -x) - Math.PI / 4;
        double rightX = -rot;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftFront.setPower(v1);
        rightFront.setPower(v2);
        leftRear.setPower(v3);
        rightRear.setPower(v4);

    }


}
