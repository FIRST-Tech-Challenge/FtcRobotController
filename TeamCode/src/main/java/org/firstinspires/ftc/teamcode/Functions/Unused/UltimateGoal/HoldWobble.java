package org.firstinspires.ftc.teamcode.Functions.Unused.UltimateGoal;

import com.qualcomm.robotcore.hardware.Servo;

// Facut de David

@Deprecated
public class HoldWobble {

    private Servo hold_wobble;

    // asta e cea mai importanta variabila:
    private boolean Status;
    /// true = merge
    /// false = oprit

    // Initializam motoarele
    public HoldWobble(Servo _HW){
        hold_wobble = _HW;
        //Status=false;
    }

    /// Le porneste
    public void Start(){

        hold_wobble.setPosition(1);
        Status=true;
    }


    /// Opreste motoarele
    public void Stop(){
        hold_wobble.setPosition(0);
        Status=false;
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
