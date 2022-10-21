package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.VoltageReader;


// Facut de Vlad


@Deprecated
public class Pistol {

    private DcMotor right_pistol;
    public VoltageReader voltageReader;

    // asta e cea mai importanta variabila:
    private boolean Status;
    /// true = merge
    /// false = oprit

    // Initializam motoarele si variabila Status
    public Pistol(DcMotor _RP, VoltageReader _VS){
        voltageReader = _VS;
        right_pistol = _RP;
        Status=false;
    }

    //porneste pistolul de la motor cu ce putere ii dai
    public void StartRaw(double Power){
        right_pistol.setPower(Power);
        Status=true;
    }

    /// Le porneste
    public void Start(){
        if(voltageReader.ReturnVoltage()>=13)
            right_pistol.setPower(-0.94);
        else
            right_pistol.setPower(-1);
        Status=true;
    }

    /// Opreste motoarele
    public void Stop(){
        // left_pistol.setPower(0);
        right_pistol.setPower(0);
        Status=false;
    }

    /// aceasta functie porneste/opreste pistolul dupa
    /// valoare lui Status
    public void Switch(){
        if(Status){
            Stop();
        }
        else{
            Start();
        }
    }
    /// verifica daca a trecut x secunde si daca da atunci schimba
    double CurrentWaitTime=0;
    double CurrentTimeStamp=0;
    public void SwitchAndWait(double x, double currentRuntime){
        if(CurrentWaitTime==0||CurrentTimeStamp+CurrentWaitTime<=currentRuntime){
            Switch();
            CurrentTimeStamp=currentRuntime;
            CurrentWaitTime=x;
        }
    }

    /// nu ai voie sa modifici variabila Status,
    /// asa ca o citesti asa:
    public boolean CheckStatus(){
        return Status;
    }


    // functia asta returneaza viteza motorului pistol
    // doar pentru debugging
    // doar trebuie pusa pur si simplu in telemetry.addData, 
    // functia genereaza textul pentru debug
    public double ReturnMotorSpeed(){
        return right_pistol.getPower();
    }

}
