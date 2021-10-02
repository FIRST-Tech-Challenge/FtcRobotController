package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**

 OpTrain

 This is the DriveTrain for TeleOp scripts.
 By putting functions here, we participate in good OOP practice.

 made with <3 by charlie gray

 */


/**

 Constructors

 **/

public class OpTrain {
    /** Variables Declaration */

    //Declare OpMode
    OpMode op;

    //Declare Motor Variables
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    private DcMotor[] motorArray = new DcMotor[4];

    //Declare Other Vars
    private double rotationPerMeter;


    /** constructors */

    /**
     * Constructor A
     *
     * @param op : OpMode class
     * @param wheelDiameter : Wheel diameter in cm
     *
     * NOTE: This constructor uses our conventional hardware names.
     *       For custom hardware names, use Constructor B
     */
    public OpTrain (OpMode op, float wheelDiameter){
        this(op,wheelDiameter,"fl","fr","bl","br");
    }


    /**
     *
     * @param op : OpMode class
     * @param wheelDiameter : Wheel diameter in cm
     * @param fl : Name of front left motor in xml
     * @param fr : Name of front right motor in xml
     * @param bl : Name of back left motor in xml
     * @param br : Name of back right motor in xml
     */
    public OpTrain(OpMode op, float wheelDiameter, String fl, String fr, String bl, String br){
        //Init op mode
        this.op = op;

        //Initialize Hardware
        front_left = op.hardwareMap.dcMotor.get(fl);
        front_right = op.hardwareMap.dcMotor.get(fr);
        back_left = op.hardwareMap.dcMotor.get(bl);
        back_right = op.hardwareMap.dcMotor.get(br);

        // reverse direction of FL and BL motors
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        motorArray[0] = front_left;
        motorArray[1] = front_right;
        motorArray[2] = back_left;
        motorArray[3] = back_right;

        for(DcMotor m:motorArray) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


        //Calculate how many full rotations of wheel to go one meter
        rotationPerMeter = 100 / (Math.pow((wheelDiameter/2),2) * Math.PI);
    }

    /** Functions */

    /**
     * travel()
     *
     * @param forwardBackPower : Power of forward back (between -1, 1)
     * @param leftRightPower : Power of left right lat (between -1, 1)
     * @param rotatePower : Power of rotation (between -1, 1)
     */
    public void travel(float forwardBackPower, float leftRightPower, float rotatePower){
        back_left.setPower(forwardBackPower - leftRightPower + rotatePower);
        front_left.setPower(forwardBackPower + leftRightPower + rotatePower);
        back_right.setPower(forwardBackPower + leftRightPower - rotatePower);
        front_right.setPower(forwardBackPower - leftRightPower - rotatePower);
    }

}
