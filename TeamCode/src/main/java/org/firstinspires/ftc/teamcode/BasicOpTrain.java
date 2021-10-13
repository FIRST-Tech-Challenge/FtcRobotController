package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**

 OpTrain

 This is the basic DriveTrain for TeleOp scripts.
 This just manages four wheel drive w/ no other functionality.
 By putting functions here, we participate in good OOP practice.

 made with <3 by charlie gray

 */


/**

 Constructors

 **/

public class BasicOpTrain {
    /** Variables Declaration */


    //Declare Motor Variables
    private DcMotor front_left = null;
    private DcMotor front_right = null;
    private DcMotor back_left = null;
    private DcMotor back_right = null;

    private DcMotor[] motorArray = new DcMotor[4];




    /** constructors */

    /**
     * Constructor
     *
     * @param fl : Front left motor
     * @param fr : Front right motor
     * @param bl : Back left motor
     * @param br : Back right motor
     */
    public BasicOpTrain(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br){

        //Initialize Hardware
        this.front_left = fl;
        this.front_right = fr;
        this.back_left = bl;
        this.back_right = br;

        // reverse direction of FL motor b/c its broken
        this.front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        this.motorArray[0] = this.front_left;
        this.motorArray[1] = this.front_right;
        this.motorArray[2] = this.back_left;
        this.motorArray[3] = this.back_right;

        for(DcMotor m:motorArray) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

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
