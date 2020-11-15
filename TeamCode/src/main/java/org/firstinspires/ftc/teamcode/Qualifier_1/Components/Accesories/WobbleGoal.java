/**
 * This is the program for moving the wobble goal arm up and down
 * @author Sid
 * @version 1.0
 * @since 2020-11-08
 * @status teleop working
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

public class WobbleGoal {

    //declaring the motor
    private DcMotor wobbleGoalMotor;
    //declaring the op mode
    private LinearOpMode op;

    public WobbleGoal(LinearOpMode opMode) {
        //setting the opmode
        this.op = opMode;

        //getting the motor from the hardware map
        wobbleGoalMotor = (DcMotor) opMode.hardwareMap.get("wobbleGoalMotor");
        //setting the zero power behavior to brake
        wobbleGoalMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //setting the direction to forward
        wobbleGoalMotor.setDirection(DcMotor.Direction.FORWARD);
        //resetting the encoder
        wobbleGoalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //setting mode to run using encoder
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
//    //making the motor turn clockwise
//    public void clockwise(){
//
//        wobbleGoalMotor.setPower(0.25);
//    }
//    //makes the motor turn counterclockwise
//    public void counterClockwise(){
//
//        wobbleGoalMotor.setPower(-0.25);
//    }
    //stops the motor
    public void stop() {
        wobbleGoalMotor.setPower(0);
    }

    public void startingPosition() {
        wobbleGoalMotor.setTargetPosition(0);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);

    }

    public void grabbingPosition() {
        wobbleGoalMotor.setTargetPosition(-940);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
    public void liftingPosition() {
        wobbleGoalMotor.setTargetPosition(-550);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
    public void droppingPosition() {
        wobbleGoalMotor.setTargetPosition(-760);
        wobbleGoalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalMotor.setPower(0.25);
    }
}
