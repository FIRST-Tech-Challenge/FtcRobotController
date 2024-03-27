package org.firstinspires.ftc.teamcode.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {


    double impulseRotation;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation){
        super(FLM, FRM, BLM, BRM);

        this.impulseRotation = impulseRotation;

    }

    public void setPowerBasic(double power){
        for(DcMotor motor : driveMotors){
            motor.setPower(power);
        }
    }
    public void mecanumDrive(float x, float y, double turn, double heading){
        double direction = Math.atan2(x, y);
        double magnitude = Math.sqrt((x*x) + (y*y));

        VectorF powerVector = new VectorF(x, y);
        rotateVector(powerVector, heading);

        float negativeGroup = (float) (powerVector.get(0) / (Math.sqrt(2) / 2));
        float positiveGroup = (float) (powerVector.get(1) / (Math.sqrt(2) / 2));

        driveMotors[0].setPower(negativeGroup + turn);
        driveMotors[1].setPower(positiveGroup - turn);
        driveMotors[2].setPower(positiveGroup + turn);
        driveMotors[3].setPower(negativeGroup - turn);

    }




    public static void rotateVector(VectorF originalVector, double angle){
        float x = originalVector.get(0);
        float y = originalVector.get(1);

        originalVector.put(0, (float) ((x * Math.cos(angle)) + ( y * Math.sin(angle))));
        originalVector.put(1, (float) ((x * -Math.sin(angle)) + (y * Math.cos(angle))));

    }

}
