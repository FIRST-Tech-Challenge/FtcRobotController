package org.firstinspires.ftc.teamcode.Qualifier_1.Accesories;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
    }

    public void shootHighGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedTopGoal * 1000);
        shooterMotor.setPower(speedTopGoal);
        op.sleep((long) sleepTime);
        shooterMotor.setPower(0);
    }

    public void shootMidGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedMediumGoal * 1000);
        shooterMotor.setPower(speedMediumGoal);
        op.sleep((long) sleepTime);
        shooterMotor.setPower(0);
    }

    public void shootLowGoal(double distance){
        this.distance=distance;
        double sleepTime = (distance / speedLowGoal * 1000);
        shooterMotor.setPower(speedLowGoal);
        op.sleep((long) sleepTime);
        shooterMotor.setPower(0);
    }
}
