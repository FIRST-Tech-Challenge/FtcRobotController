package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LinearSlideSub extends SubsystemBase {

    Telemetry telemetry;

    public DcMotor linearSlideMotor;

    Boolean previouslyDown; // whether the arm was down last time move was ran.

    public LinearSlideSub(HardwareMap hardwareMap, Telemetry tm) {
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        this.telemetry = tm;

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Slide Height", linearSlideMotor.getCurrentPosition());
    }

    public DcMotor getMotor(){
        return linearSlideMotor;
    }

    /**
     * Reset the encoder
     */
    public void resetEncoder(){
        this.linearSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //         > 6000 + going up   ->    N
    //         > 6000 + going down ->    Y
    //         < 6000 + going up   ->    Y
    //         < 6000 + going down ->    Y
    public void move(double speed) {
        // Stop motor if above top limit
        if (linearSlideMotor.getCurrentPosition() > 6000 && speed > 0) {
            linearSlideMotor.setPower(0);
        }else if (linearSlideMotor.getCurrentPosition()<=0) { // if the arm is all the way down
            if (speed > 0) { // if the arm is moving up + its all the way down
                linearSlideMotor.setPower(speed);
            } else { // if the arm is moving down + its all the way down
                linearSlideMotor.setPower(0);
            }
        } else { // if the arm is not all the way down.
            linearSlideMotor.setPower(speed);
        }
    }


    //special move command that doesn't adhere to limits
    public void moveNoLimit(double speed) {
        linearSlideMotor.setPower(speed);
    }




}