package org.firstinspires.ftc.teamcode.Functions.Unused.FreightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.MoveAutocorrect2;

import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;

@TeleOp(name="ArmEncoder", group="TEST")
public class ArmEncoderTest extends OpMode {

    /**
     * Initialize the motors.
     */
    private DcMotorEx armMotorLeft, armMotorRight;
    private MoveAutocorrect2 AutoCorrection;
    private RotationDetector rotationDetector;
    private Move move;
    private Rotate rotate;

    /**
     * Initialize the variables that store the current position of the encoders.
     */

    private int leftBackPos;
    private int rightBackPos;
    @Override
    public void init() {
        AutoCorrection = new MoveAutocorrect2(rotationDetector,move,rotate);
        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");


        /**
         * Resets the encoders to 0 position.
         */
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /**
         * Reverse the motor so the drive wheels don't spin in opposite direction.
         */
        armMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /**
         * Initialize the encoders position to 0.
         */
        leftBackPos = 0;
        rightBackPos = 0;

    }

    @Override
    public void loop() {
        if(gamepad2.a) drive(-100, -100,0.5);
    }

    private void drive(int leftTarget, int rightTarget, double speed){

        leftBackPos += leftTarget;
        rightBackPos += rightTarget;


        /**
         * Set the target position, and set the mode to run to position.
         * This will make encoders move until they get to the target position.
         */
        armMotorRight.setTargetPosition(leftBackPos);
        armMotorLeft.setTargetPosition(rightBackPos);

        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotorRight.setPower(speed);
        armMotorLeft.setPower(speed);

        while ( armMotorRight.isBusy() && armMotorLeft.isBusy()){
            //idle();
        }
    }
}
