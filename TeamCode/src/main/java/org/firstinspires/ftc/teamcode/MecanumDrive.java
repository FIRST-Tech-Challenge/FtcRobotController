package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class MecanumDrive extends DriveTrain {
    double impulseRoation;

    public MecanumDrive(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation){
        super(FLM, FRM, BLM, BRM);
        this.impulseRoation = impulseRotation;
    }
    public void drive(float forward, float sideway, double turn){
        VectorF powerVector = new VectorF(forward, sideway);
        //rotateVector(powerVector, impulseRoation);

        float negativeGroup = (float) (powerVector.get(0) / (Math.sqrt(2) / 2));
        float positiveGroup = (float) (powerVector.get(1) / (Math.sqrt(2) / 2));

        driveMotors[0].setPower(negativeGroup + turn);
        driveMotors[1].setPower(positiveGroup - turn);
        driveMotors[2].setPower(positiveGroup + turn);
        driveMotors[3].setPower(negativeGroup - turn);

    }




//    public static VectorF rotateVector(VectorF originalVector, double angle){
//        float x = originalVector.get(0);
//        float y = originalVector.get(1);
//
//        originalVector.put(0, (float) ((x * Math.cos(angle)) + ( y * Math.sin(angle))));
//        originalVector.put(1, (float) ((x * -Math.sin(angle)) + (y * Math.cos(angle))));
//
//        return originalVector;
//    }

}
