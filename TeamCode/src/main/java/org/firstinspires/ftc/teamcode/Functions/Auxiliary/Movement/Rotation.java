package org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Rotation extends MovementFunction{
    /**
     * This method initialises the motors. It will set their ZeroPowerBehavior to ZeroPowerBehavior.BRAKE
     *
     * @param leftFrontMotor  : DcMotor left front
     * @param rightFrontMotor : DcMotor right front
     * @param leftBackMotor   : DcMotor left back
     * @param rightBackMotor  : DcMotor right back
     */
    public Rotation(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftBackMotor, DcMotor rightBackMotor) {
        super(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
    }

    @Override
    void MoveCode(Move direction, double power) {
        switch(direction){
            case LEFT:
                leftFrontMotor.setPower(power);
                leftBackMotor.setPower(power);
                rightBackMotor.setPower(power);
                rightFrontMotor.setPower(power);
                break;
            case RIGHT:
                leftFrontMotor.setPower(-power);
                leftBackMotor.setPower(-power);
                rightBackMotor.setPower(-power);
                rightFrontMotor.setPower(-power);
                break;
        }
    }
}
