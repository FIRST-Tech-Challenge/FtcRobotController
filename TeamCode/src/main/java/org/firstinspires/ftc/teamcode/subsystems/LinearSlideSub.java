package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LinearSlideSub extends SubsystemBase {

    Telemetry telemetry;

    private DcMotor linearSlideMotor;

    public LinearSlideSub(HardwareMap hardwareMap, Telemetry tm) {
        linearSlideMotor = hardwareMap.get(DcMotor.class, "linearSlideMotor");
        this.telemetry = tm;

        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Slide Position", linearSlideMotor.getCurrentPosition());
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
        this.linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void move(double speed) {
        if ((getMotor().getCurrentPosition() > Constants.LinearSlideConstants.upwardLimit) && (speed > 0)) {
            speed = 0;
        } else if ((getMotor().getCurrentPosition() < Constants.LinearSlideConstants.downwardLimit) && (speed < 0)) {
            speed = 0;
        }

        linearSlideMotor.setPower(speed);
    }


    //special move command that doesn't adhere to limits
    //TODO: Maybe should replace final line of move? They're the same.
    public void moveNoLimit(double speed) {
        linearSlideMotor.setPower(speed);
    }




}