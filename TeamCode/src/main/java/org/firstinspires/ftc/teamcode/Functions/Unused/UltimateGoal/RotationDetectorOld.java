package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;


public class RotationDetectorOld {




    // Aceasta functie se initializeaza astfel:
    // rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));

    /*
    -------------------------------------
    TEMPLATE CUM SA FOLOSESTI CLASA ASTA:
    -------------------------------------
    int Unghi = <--- aici pui unghiul
    while(rotationDetector.WaitForRotation(Unghi))
    {
        rotate.RotateRaw(1, rotationDetector.VitezaMotoare(Unghi));
    }
    rotate.MoveStop(); // aceasta linie e doar de siguranta.

    */

    double StartingRotation=0;

    BNO055IMU Gyro;
    public RotationDetectorOld(BNO055IMU _Gyro){
        Gyro=_Gyro;
        BNO055IMU.Parameters par = new BNO055IMU.Parameters();
        par.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        par.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Gyro.initialize(par);
        while(!Gyro.isGyroCalibrated()){

        }
        StartingRotation=ReturnPositiveRotation();

    }

    public BNO055IMU ReturnGyro(){
        return Gyro;
    }

    public double ReturnStartingRotation(){
        return StartingRotation;
    }

    public double ReturnRotation(){
        return Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }



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

    public boolean WaitForRotation(int TargetRotation){
        if(CorectieUnghi(TargetRotation)==CorectieUnghi((int)ReturnPositiveRotation()))
        {
            return false;
        }
        if(TargetRotation-1<=(int)ReturnPositiveRotation()
                && TargetRotation+1>=(int)ReturnPositiveRotation())
        {
            return false;
        }
        return true;
    }


    // functia asta se foloseste un pic altfel: se pune intr-un while ca conditia de executie si va iesi din while
    // cand se invarte pana la pozitia data

    // functia asta iti calculeaza directia simuland rotatia robotului in cele doua directii si vazand cu, ajunge mai rapid la destinatie
    int CalcDirectie(int UnghiDestinatie) {
        int UnghiCurrent= (int) ReturnPositiveRotation();
        MVVariables.Vector2 Directii = new MVVariables.Vector2(UnghiCurrent, UnghiCurrent);
        for(int i=1;i<=180;i++){
            Directii= new MVVariables.Vector2(CorectieUnghi(UnghiCurrent+i), CorectieUnghi(UnghiCurrent-i));
            if(Directii.x==UnghiDestinatie){
                return 1;
            }
            else if(Directii.y==UnghiDestinatie){
                return -1;
            }
        }
        return 1;
    }

    public double VitezaMotoare(int TargetRotation){
        //calculam distanta
        int modifier= CalcDirectie(TargetRotation);
        double distantaPrim = Math.abs(TargetRotation-(int)ReturnPositiveRotation());
        double distantaSecund = 360 - distantaPrim;
        // luam doar distanta cea mai mica;
        double distantaFinala=0;
        if(distantaPrim >= distantaSecund){
            distantaFinala = distantaSecund;
        }
        else{
            distantaFinala = distantaPrim;
        }
        distantaFinala = Math.abs(distantaFinala);
        // inainte de calcula viteza finala trebuie sa impunem minimul distantei pentru impartire pentru
        // cazul in care distanta e prea mica si trebuie mers incet
        double vitezaDelta=distantaFinala;
        if(distantaFinala<=15 && distantaFinala>1){
            return 0.3*modifier;
        }
        else if(distantaFinala<=1){
            return 0.2*modifier;
        }
        else if(distantaFinala==0){
            return 0;
        }
        else
        {
            return 1*modifier;
        }
    }



    // functia asta se ocupa ca daca cumva cineva baga un nr peste 360 sa il corecteze

    public int CorectieUnghi(int Unghi){
        int UnghiAux=Unghi;
        while(UnghiAux<0){
            UnghiAux=UnghiAux+360;
        }
        while(UnghiAux>=360){
            UnghiAux=UnghiAux-360;
        }
        return UnghiAux;
    }

    int LastAngleReported=0;

    //functia returneaza true atata timp cat robotul se roteste
    public boolean IsRotating(){
        if(LastAngleReported!=(int)ReturnRotation()){
            LastAngleReported=(int)ReturnRotation();
            return true;
        }
        return false;
    }

}
