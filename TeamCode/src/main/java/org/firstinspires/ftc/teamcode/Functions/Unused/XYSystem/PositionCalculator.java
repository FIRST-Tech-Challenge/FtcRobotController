package org.firstinspires.ftc.teamcode.Functions.Unused.XYSystem;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.AccelerationDetector;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.RotationDetector;
import org.firstinspires.ftc.teamcode.Functions.VoltageReader;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

import java.io.IOException;
import java.lang.Math;


@Deprecated
public class PositionCalculator {
    public Move move;
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private RotationDetector rotationDetector;
    private VoltageReader voltageReader;
    public AccelerationDetector accelerationDetector;
    public MVVariables.Vector2 Position = new MVVariables.Vector2(0, 0);


    public PositionCalculator(Move _move,
                              double _x, double _y, RotationDetector _RotationDetector, VoltageReader _VoltageReader) throws IOException {
        leftMotor = _move.ReturnMotor(1);
        rightMotor = _move.ReturnMotor(2);
        leftMotorBack = _move.ReturnMotor(3);
        rightMotorBack = _move.ReturnMotor(4);
        move = _move;
        rotationDetector = _RotationDetector;
        voltageReader = _VoltageReader;
        accelerationDetector = new AccelerationDetector(_RotationDetector.ReturnGyro());
        Position.x = _x;
        Position.y = _y;

    }

    //calibreaza accelerometrul referinta completa la functia Calibration() din AccelerationDetector
    public String CalibrateAccel(){
        return accelerationDetector.Calibration();
    }

    MVVariables.Vector2 Accel;
    double lastruntime=0;
    public String NewUpdate(double runtime){
        accelerationDetector.UpdateAccel();
        Accel = new MVVariables.Vector2(accelerationDetector.rawAccel.x, accelerationDetector.rawAccel.y);
        Accel.MultiplyNumber((runtime - lastruntime));
        //Accel.MultiplyNumber((runtime - lastruntime));
        lastruntime = runtime;
        //Accel.MultiplyNumber(100);
        String debugData ="Debug data:\n" + "Position Vector: "
                + Position.ReturnData() + "\nAccel Vector: "+ Accel.ReturnData() + " lastruntime: " + lastruntime + " runtime: "
                + runtime + " move.ReturnCurrentDirection(): "+ move.ReturnCurrentDirection();
        long adaus=0;
        MVVariables.Vector2 VModifier = new MVVariables.Vector2(1, 1);
        //move.ReturnCurrentDirection()*90
        switch ((int) move.ReturnCurrentDirection()){
            case 0: adaus=-1;
                accelerationDetector.realAccel.Reset();
                break;
            case 1: adaus=0;
                break;
            case 2: adaus=0;
                VModifier.NegativeX();
                break;
            case 3: adaus=0;
                VModifier.NegativeY();
                break;
            case 4: adaus=0;
                break;
        }
        if (Accel.TranformToNewPlane(rotationDetector.ReturnPositiveRotation())) {

            /*

            switch ((int) move.ReturnCurrentDirection()) {
                case 1:
                    if(Position.y<0)
                        Position.NegativeX();
                    Position.AddVector(Accel);
                case 3:
                    if(Position.x<0)
                        Position.NegativeY();
                    Position.AddVector(Accel);
                    break;
                case 2:
                    if(Position.x>0)
                        Position.NegativeX();
                    Position.AddVector(Accel);
                    break;
                case 4:
                    if(Position.y>0)
                        Position.NegativeY();
                    Position.AddVector(Accel);
                    break;

            }
            */
            //Accel.MultiplyNumber(VModifier.x);
            if(adaus!=-1){
                Position.AddVector(Accel);}
            return getClass().getName()+" - Works Well. " + debugData;
        }
        else {
            Accel.MultiplyNumber(VModifier.x);
            if(adaus!=-1) {
                Position.AddVector(Accel);
            }
            return getClass().getName()+" - ERROR. " + debugData;
        }

    }

    double lastruntime2=0;
    public String Update(double runtime){
        accelerationDetector.UpdateAccel();
        MVVariables.Vector2 Accel = new MVVariables.Vector2(accelerationDetector.realAccel.x, accelerationDetector.realAccel.y);
        Accel.MultiplyNumber((runtime - lastruntime));
        lastruntime = runtime;
        Accel.MultiplyNumber(100);
        String debugData ="Debug data:\n" + "Position Vector: "
                + Position.ReturnData() + "\nAccel Vector: "+ Accel.ReturnData() + " lastruntime: " + lastruntime + " runtime: "
                + runtime;
        if (Accel.TranformToNewPlane(rotationDetector.ReturnPositiveRotation()+move.ReturnCurrentDirection()*90)) {


            /*

            switch ((int) move.ReturnCurrentDirection()) {
                case 1:
                    if(Position.y<0)
                        Position.NegativeX();
                    Position.AddVector(Accel);
                case 3:
                    if(Position.x<0)
                        Position.NegativeY();
                    Position.AddVector(Accel);
                    break;
                case 2:
                    if(Position.x>0)
                        Position.NegativeX();
                    Position.AddVector(Accel);
                    break;
                case 4:
                    if(Position.y>0)
                        Position.NegativeY();
                    Position.AddVector(Accel);
                    break;

            }
            */
            return getClass().getName()+" - Works Well. " + debugData;
        }
        else {
            return getClass().getName()+" - ERROR. " + debugData;
        }
    }


    public String ReturnAccelData(){
        return "Position Vector: "
                + Position.ReturnData() + "\nAccel Vector: "+ Accel.ReturnData() + " lastruntime: " + lastruntime + " move.ReturnCurrentDirection(): "+ move.ReturnCurrentDirection();
    }

    public String ReturnData(){
        return "Accel status: "+ReturnAccStatus()+"\nX:"+ReturnX()+"\nY:"+ReturnY();
    }

    // angle offset is used if going down or left (-1) or up and right (1)
    /*public void Update(double AngleOffset){
        //if(!rotationDetector.IsRotating()){
            {
                accelerationDetector.StartAcc();
                CalcularePoz(accelerationDetector.ReturnDistanta()*AngleOffset, CorectieUnghi(rotationDetector.ReturnRotation()));
                if(Double.isNaN(Position.x)){
                    Position.x=0;
                }
                if(Double.isNaN(Position.y)){
                    Position.y=0;
                }
                if(!Double.isNaN(LaturaX) && !Double.isNaN(LaturaY)){
                    switch (Cadran) // adaptam rezultatul final dupa cadran
                    {
                    case 1:
                        Position.x = Position.x + LaturaX;
                        Position.y = Position.y + LaturaY;
                        break;
                    case 2:
                        Position.x = Position.x - LaturaY;
                        Position.y = Position.y + LaturaX;
                    // CurrentPosition += new Vector2(-LaturaY, LaturaX); // e invers
                        break;
                    case 3:
                        Position.x = Position.x - LaturaX;
                        Position.y = Position.y - LaturaY;
                    // CurrentPosition += new Vector2(-LaturaX, -LaturaY);
                        break;
                    case 4:
                        Position.x = Position.x + LaturaY;
                        Position.y = Position.y - LaturaX;
                    // CurrentPosition += new Vector2(LaturaY, -LaturaX); // e invers de ce? D-AIA
                        break;
                    }
                }
                accelerationDetector.StopAcc();
            }
        //}
    }*/

    public double ReturnDistanta(){
        return accelerationDetector.ReturnDistance();
    }
    public double ReturnXAcc(){
        return accelerationDetector.ReturnX();
    }
    public double ReturnYAcc(){
        return accelerationDetector.ReturnY();
    }
    public double ReturnZAcc(){
        return accelerationDetector.ReturnZ();
    }
    public String ReturnAccStatus(){
        return accelerationDetector.AccelerometerStatus();
    }

    public double ReturnX(){
        return Position.x*100;
    }

    public double ReturnY(){
        return Position.y*100;
    }

    public double ReturnTestX(){
        return accelerationDetector.ReturnTestX();
    }

    public double ReturnTestY(){
        return accelerationDetector.ReturnTestY();
    }

    public double ReturnTestZ(){
        return accelerationDetector.ReturnTestZ();
    }

    int direction=0; //0 stop, 1 fata/spate, 2 lateral, 3 rotate

    int CalculareDirectie(int _D){
        if(_D<=2){
            return 1;
        }
        else{
            return 2;
        }
    }



    public double LaturaY;
    public double LaturaX;
    public int Cadran;

    // functia asta calculeaza pozitia
    // returneaza false daca are eroare, altfel returneaza true
    void CalcularePoz(double _Distanta, double Unghi){
        Cadran = CadranUnghi(Unghi);
        double x = CorectieUnghi(Unghi);
        double y = CorectieUnghi(90 - x);

        // transformam din grade in radian
        double radx = Math.toRadians(x);
        double rady = Math.toRadians(y);

        // calculam laturile folosind radianul x
        LaturaY = Math.sin(radx) * _Distanta;
        LaturaX = Math.cos(radx) * _Distanta;

        // calculam laturile folosind radianul y pentru verificare
        double VerificareLaturaY = Math.cos(rady) * _Distanta;
        double VerificareLaturaX = Math.sin(rady) * _Distanta;

        // scoatem toate zecimalele dupa pozitia 5 deoarece avem probleme la verificare altfel
        // pentru ca fix ultima zecimala nu e egala si gata tot codul

        LaturaY = RemoveDecimal(LaturaY, 3);
        LaturaX = RemoveDecimal(LaturaX, 3);
        //VerificareLaturaY = RemoveDecimal(VerificareLaturaY, 5);
        //VerificareLaturaX = RemoveDecimal(VerificareLaturaX, 5);


        // verificarea
        /*
        double ErrorMargin = 0.1f;
        if (Math.abs(LaturaY - VerificareLaturaY) >= ErrorMargin) // verificam latura y
        {
            // strigam dupa ajutor ca avem buba mare
            // sau nu strigam, dar avem buba mare
            return false;
        }
        else if (Math.abs(LaturaX - VerificareLaturaX) >= ErrorMargin) // verificam latura x
        {
            // strigam dupa ajutor ca avem buba mare
            // sau nu strigam, dar avem buba mare
            return false;
        }
        
        else*/ // daca a trecut de cele doua verificari, trecem mai departe




    }


    /// <summary>
    /// functia asta scoate decimalele dupa pozitia p
    /// faptul ca nu este o functie de genul asta in math de la java ma depaseste
    /// </summary>
    /// <returns> numarul number dar cu p zecimale <returns>
    double RemoveDecimal(double number, int p)
    {
        int i = 0;
        double newNumber = number;
        for(i = 0; i<=p; i++)
        {
            newNumber = newNumber * 10;
        }
        newNumber = (int)newNumber;
        for(i=0;i<=p;i++)
        {
            newNumber = newNumber / 10;
        }
        return newNumber;
    }



    int CadranUnghi(double Unghi){
        int Cadran = 1;
        double NewAngle = Unghi;
        while (NewAngle - 90 >= 0)
        {
            NewAngle -= 90;
            Cadran++;
        }
        return Cadran;
    }
    double IncadrareUnghi(double Unghi){
        int Cadran = 1;
        double NewAngle = Unghi;
        while (NewAngle - 90 >= 0)
        {
            NewAngle -= 90;
            Cadran++;
        }
        return NewAngle;
    }


    void Stop(){
        move.MoveStop();
    }


    public double CorectieUnghi(double Unghi){
        double UnghiAux=Unghi;
        while(UnghiAux>=360){
            UnghiAux=UnghiAux-360;
        }
        return UnghiAux;
    }




    // todo: write your code here
}
