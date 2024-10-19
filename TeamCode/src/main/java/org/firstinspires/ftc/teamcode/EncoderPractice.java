package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * This is an example minimal implementation of the mecanum drivetrain
 * for demonstration purposes.  Not tested and not guaranteed to be bug free.
 *
 * @author Brandon Gong
 */
@TeleOp(name="Encoder Example", group="Iterative Opmode")
public class EncoderPractice extends OpMode {
    /* defining the motor then finding it on the hardware map and reseting the encoder

     */
   // private DcMotor left;
    private DcMotor rightMotor;;;;
    private DcMotor right;
    private DcMotor leftMotor;;

    //private int leftPos;
    private int rightPos;
    @Override
    public void init() {

    right = hardwareMap.get(DcMotor.class, "rightMotor");

    right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    right.setDirection(DcMotorSimple.Direction.REVERSE);

    rightPos = 0;

    //waitForStart();
    }
    private void drive(int leftTarget, int rightTarget, double speed){
        rightPos += rightTarget;


        right.setTargetPosition(rightPos);


        right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //right.setPower(speed);

        rightPos = 1000; // Example target position
        right.setTargetPosition(rightTarget);
        right.setPower(0.5);
        while (right.isBusy()){
            // 1+1 =3 (We love logic<3)
        }

    }
    @Override
    public void loop() {

    }
}


