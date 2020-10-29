package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * <h1> Ultimate Goal Accessory
 * <p>
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */
public class Shooter {
    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    public DcMotor shooterMotor;
    Servo shooter_Servo;
    private float speedTopGoal = 100;//will get changed when testing
    private float speedMediumGoal=80;//will get changed when testing
    private float speedLowGoal=50;//will get changed when testing
    private double distance;


    public Shooter(){

    }

    public void initChassis(LinearOpMode opMode) {//initialization
        op = opMode;
        hardwareMap = op.hardwareMap;

        shooterMotor = hardwareMap.dcMotor.get("ShooterMotor");//gets the name ShooterMotor from hardware map and assigns it to shooter_Motor
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int Gobilda_Ticks_Per_Rev=28;
    }

    public void moveServo(boolean direction){
        if (direction == true){
            shooter_Servo.setPosition(1.0);
        }

        else{
            shooter_Servo.setPosition(0.0);
        }
    }

    public void shootHighGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedTopGoal * 1000);
        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooterMotor.setTargetPosition(20000);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setPower(speedTopGoal);

        while (shooterMotor.isBusy()) {
            moveServo(true);
            moveServo(false);
            moveServo(true);
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);

    }

    public void shootMidGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedMediumGoal * 1000);
        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooterMotor.setTargetPosition(20000);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setPower(speedMediumGoal);

        while (shooterMotor.isBusy()) {
            moveServo(true);
            moveServo(false);
            moveServo(true);
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);
    }

    public void shootLowGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedLowGoal * 1000);
        shooterMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);

        shooterMotor.setTargetPosition(20000);

        shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        shooterMotor.setPower(speedLowGoal);

        while (shooterMotor.isBusy()) {
            moveServo(true);
            moveServo(false);
            moveServo(true);
            moveServo(false);
            moveServo(true);
        }
        shooterMotor.setPower(0);
    }


}
