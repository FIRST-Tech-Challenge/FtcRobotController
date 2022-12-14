package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

// facut de Vlad Durdeu
public class RotateMove {

    DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    BNO055IMU Gyro;

    // initializeaza motoarele astfel incat sa franeze cand le da valoarea 0
    void Init(){
        BNO055IMU.Parameters par = new BNO055IMU.Parameters();
        par.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        par.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gyro.initialize(par);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        while(!Gyro.isGyroCalibrated()){
        }
    }

    public RotateMove(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB, BNO055IMU _Gyro){
        leftMotor = _LMF;
        rightMotor = _RMF;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;
        Gyro= _Gyro;
        Init();
    }

    // folosit in ReturnPositiveRotation
    public double ReturnRotation(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    // folosit in DebugData
    public double ReturnPositiveRotation(){
        double CurrentRotation=ReturnRotation();
        if(CurrentRotation>=0)
        {
            return CurrentRotation;
        }
        else
        {
            return 360+CurrentRotation;
        }

    }

    double CurrentAngle=0;

    public String DebugData(){
        return "ReturnPositiveRotation:"+ ReturnPositiveRotation() 
                +"\nCurrentAngle:"+ CurrentAngle +"\nleftMotor = "+leftMotor.getPower()+" \nrightMotor = "+rightMotor.getPower()
                +"\nleftMotorBack = "+leftMotorBack.getPower()+"\nrightMotorBack = "+rightMotorBack.getPower();
    }


    // 1 = stanga fata
    // 2 = dreapta fata
    // 3 = stanga spate
    // 4 = dreapta spate
    void MoveRaw(int direction, double movePower, double rotationPower){
        try{
            if(movePower<rotationPower){
                movePower=rotationPower;
            }
            switch(direction){
                //merge in fata
                case 1: // Se roteste in stanga , deci totul trebuie sa fie spre pozitiv
                    leftMotor.setPower(-movePower+rotationPower);
                    leftMotorBack.setPower(-movePower+rotationPower);
                    rightMotorBack.setPower(movePower);
                    rightMotor.setPower(movePower);
                    break;
                case 2: // Se roteste in dreapta, deci totul trebuie sa fie spre negativ
                    leftMotor.setPower(-movePower);
                    leftMotorBack.setPower(-movePower);
                    rightMotorBack.setPower(movePower-rotationPower);
                    rightMotor.setPower(movePower-rotationPower);
                    break;

                //merge in spate
                case 3: // Se roteste in stanga, deci totul trebuie sa fie spre pozitiv
                    leftMotor.setPower(movePower);
                    leftMotorBack.setPower(movePower);
                    rightMotorBack.setPower(-movePower+rotationPower);
                    rightMotor.setPower(-movePower+rotationPower);
                    break;
                case 4: // Se roteste in dreapta, deci totul trebuie sa fie spre negativ
                    leftMotor.setPower(movePower-rotationPower);
                    leftMotorBack.setPower(movePower-rotationPower);
                    rightMotorBack.setPower(-movePower);
                    rightMotor.setPower(-movePower);
                    break;


            }
        }
        catch(Exception e) {}
    }

    // opreste motoarele
    public void MoveStop(){
        leftMotor.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);
        rightMotor.setPower(0);
    }

    public void MoveFull(int direction, double rotationPower){
        MoveRaw(direction, 1, rotationPower);
    }

    //facut de Toni
    public void CurveBy(int direction){
        switch (direction){
            //merge in fata
            case 1: // fata stanga = scad puterea motoarelor din stanga
                leftMotor.setPower(-0.5);
                leftMotorBack.setPower(-0.5);
                rightMotor.setPower(1);
                rightMotorBack.setPower(1);
                break;
            case 2: //fata dreapta = scad puterea motoarelor din dreapta
                leftMotor.setPower(-1);
                leftMotorBack.setPower(-1);
                rightMotor.setPower(0.5);
                rightMotorBack.setPower(0.5);
                break;

            //merge in spate
            case 3: //spate stanga = scad puterea motoarelor din stanga
                leftMotor.setPower(0.5);
                leftMotorBack.setPower(0.5);
                rightMotor.setPower(-1);
                rightMotorBack.setPower(-1);
                break;
            case 4: //spate dreapta = scad puterea motoarelor din dreapta
                leftMotor.setPower(1);
                leftMotorBack.setPower(1);
                rightMotor.setPower(-0.5);
                rightMotorBack.setPower(-0.5);
                break;

            //merge in dreapta
            case 5: //dreapta fata = scad puterea motoarelor din fata
                leftMotor.setPower(-0.5);
                leftMotorBack.setPower(-1);
                rightMotor.setPower(-0.5);
                rightMotorBack.setPower(-1);
                break;
            case 6: //dreapta spate = scad puterea motoarelor din spate
                leftMotor.setPower(-1);
                leftMotorBack.setPower(-0.5);
                rightMotor.setPower(-1);
                rightMotorBack.setPower(-0.5);
                break;

            //merge in stanga
            case 7: //stanga fata = scad puterea motoarelor din fata
                leftMotor.setPower(0.5);
                leftMotorBack.setPower(1);
                rightMotor.setPower(0.5);
                rightMotorBack.setPower(1);
                break;
            case 8: //stanga spate = scad puterea motoarelor din spate
                leftMotor.setPower(1);
                leftMotorBack.setPower(0.5);
                rightMotor.setPower(1);
                rightMotorBack.setPower(0.5);
                break;
        }
    }

}
