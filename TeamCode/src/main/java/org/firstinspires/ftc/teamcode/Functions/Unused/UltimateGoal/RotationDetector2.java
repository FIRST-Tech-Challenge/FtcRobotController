package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Deprecated
public class RotationDetector2 {

    // Aceasta functie se initializeaza astfel:
    // rotationDetector = new RotationDetector(hardwareMap.get(BNO055IMUImpl.class, "imu"));

    /*
    -------------------------------------
    TEMPLATE CUM SA FOLOSESTI CLASA ASTA:
    -------------------------------------
    int Unghi = <--- aici pui unghiul
    while(rotationDetector.WaitForRotation(Unghi))
    {
        rotate.RotateRaw(1, rotationDetector.VitezaMotoareNou(Unghi));
    }
    rotate.MoveStop(); // aceasta linie e doar de siguranta.


    // template old - nu mai folosim templateul asta
    while(rotationDetector.WaitForRotation(Unghi))
    {
        rotate.RotateRaw(rotationDetector.Directie(Unghi), rotationDetector.VitezaMotoare(Unghi));
    }
    rotate.MoveStop();
    */

    double StartingRotation=0;

    BNO055IMU Gyro;
    public RotationDetector2(BNO055IMU _Gyro){
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


    // functia asta se foloseste un pic altfel: se pune intr-un while ca conditia de executie si va iesi din while
    // cand se invarte pana la pozitia data



    // se scrie target rotation

    public boolean WaitForRotation(int TargetRotation){
        //TargetRotation-1<=ReturnPositiveRotation()&&ReturnPositiveRotation()<=TargetRotation+1
        //
        if(CorectieUnghi(TargetRotation-3)<=(int)ReturnPositiveRotation() && CorectieUnghi(TargetRotation+3)>=(int)ReturnPositiveRotation())
        {
            return false;
        }
        return true;
    }

    // functia asta va returna 1 pentru stanga si 2 pentru dreapta
    @Deprecated
    public int Directie(int TargetRotation){
        if(0 <= CorectieUnghi(TargetRotation-(int)ReturnPositiveRotation())
                && CorectieUnghi(TargetRotation-(int)ReturnPositiveRotation())
                <= 180){
            return 1;
        }
        else
        {
            return 2;
        }
    }

    // functia asta incetineste motoarele a.i sa se ajunga la fix unde trebuie
    // returneaza viteza motoarelor
    @Deprecated
    public double VitezaMotoare(int TargetRotation){
        //calculam distantele
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
        if(distantaFinala<=15){
            return 0.3;
        }
        else
        {
            return 1;
        }

    }

    // ideea acestei functii este sa roteasca robotul folosind viteza in loc sa calculeze directia
    // daca ii zicem la robot sa se roteasca stanga dar ii dam valori negative, se va roti dreapta
    @Deprecated
    public double VitezaMotoareNou(int TargetRotation){
        //calculam distanta
        int modifier=1;
        double distantaPrim = TargetRotation-(int)ReturnPositiveRotation();
        if(distantaPrim==0){
            modifier*=-1;
        }
        // luam doar distanta cea mai mica;
        double distantaFinala=0;
        if(distantaPrim <= 359-distantaPrim){
            distantaFinala = distantaPrim;
            modifier*=1;
        }
        else{
            modifier*=-1;
            distantaFinala = 360-distantaPrim;
        }
        distantaFinala = Math.abs(distantaFinala);
        // inainte de calcula viteza finala trebuie sa impunem minimul distantei pentru impartire pentru
        // cazul in care distanta e prea mica si trebuie mers incet
        double vitezaDelta=distantaFinala;
        if(distantaFinala<=15 && distantaFinala>8){
            return 0.2*modifier;
        }
        else if(distantaFinala<=8 && distantaFinala>2){
            return 0.1*modifier;
        }
        else if(distantaFinala<=2){
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
