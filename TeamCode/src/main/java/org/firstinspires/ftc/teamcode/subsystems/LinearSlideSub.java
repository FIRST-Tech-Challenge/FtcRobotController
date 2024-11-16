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

    public void move(double speed) {
        // Stop motor if above top limit
        if (speed > 0) { // if the arm is moving up + its all the way down
            linearSlideMotor.setPower(speed);
        } else { // if the arm is moving down + its all the way down
            linearSlideMotor.setPower(0);
        }
    }


    //special move command that doesn't adhere to limits
    public void moveNoLimit(double speed) {
        linearSlideMotor.setPower(speed);
    }




}