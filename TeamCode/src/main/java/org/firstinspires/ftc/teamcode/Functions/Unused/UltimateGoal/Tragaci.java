package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.robotcore.hardware.Servo;

// Facut de Vlad


@Deprecated
public class Tragaci {

    private Servo pushRing;

    // asta e cea mai importanta variabila:
    private boolean Status;
    /// true = merge
    /// false = oprit

    // Initializam motoarele si variabila Status
    public Tragaci(Servo _PushRing){
        pushRing = _PushRing;
        Status=false;
    }

    /// Le porneste
    public void Start(){
        pushRing.setPosition(0.55);
        Status=true;
    }

    /// Opreste servoul
    public void Stop(){
        pushRing.setPosition(0.68);
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


}
