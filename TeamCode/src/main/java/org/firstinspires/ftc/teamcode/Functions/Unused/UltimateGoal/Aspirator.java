package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.robotcore.hardware.DcMotor;

// Facut de Vlad
@Deprecated
public class Aspirator {

    private DcMotor vaccumMotor;

    // asta e cea mai importanta variabila:
    private boolean Status;
    /// true = merge
    /// false = oprit

    // asta e cea mai importanta variabila:
    private boolean StatusInv;
    /// true = merge
    /// false = oprit

    // Initializam motoarele
    public Aspirator(DcMotor _VC){
        vaccumMotor = _VC;

        Status=false;
        StatusInv=false;
    }

    /// Le porneste
    public void Start(){
        vaccumMotor.setPower(-0.95);
        Status=true;
        StatusInv=false;
    }

    /// Le porneste invers
    public void StartInvers(){
        vaccumMotor.setPower(1);
        StatusInv=true;
        Status=false;
    }

    /// Opreste motoarele
    public void Stop(){
        vaccumMotor.setPower(0);
        Status=false;
        StatusInv=false;
    }

    /// aceasta functie porneste/opreste aspiratorul dupa
    /// valoare lui Status
    public void Switch(){
        if(Status){
            Stop();
        }
        else{
            Start();
        }
    }

    /// aceasta functie porneste/opreste aspiratorul dupa
    /// valoare lui Status
    /// INVERS
    public void SwitchInvers(){
        if(StatusInv){
            Stop();
        }
        else{
            StartInvers();
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

    /// verifica daca a trecut x secunde si daca da atunci schimba
    /// INVERS
    double CurrentWaitTimeInv=0;
    double CurrentTimeStampInv=0;
    public void SwitchAndWaitInv(double x, double currentRuntime){
        if(CurrentWaitTimeInv==0||CurrentTimeStampInv+CurrentWaitTimeInv<=currentRuntime){
            SwitchInvers();
            CurrentTimeStampInv=currentRuntime;
            CurrentWaitTimeInv=x;
        }
    }

    /// nu ai voie sa modifici variabila Status,
    /// asa ca o citesti asa:
    public boolean CheckStatus(){
        return Status;
    }


}
