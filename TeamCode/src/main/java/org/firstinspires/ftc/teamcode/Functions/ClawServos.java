package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.Servo;

public class ClawServos {

    private Servo leftServo, rightServo;
    private boolean status1;
    public ClawServos(Servo _LS, Servo _RS)
    {
        leftServo = _LS;
        rightServo = _RS;
        status1 = false;
    }

    public void Open()
    {
        leftServo.setPosition(1);
        rightServo.setPosition(1);
        status1 = true;
    }
    public void Close() {
        leftServo.setPosition(0);
        rightServo.setPosition(0);
        status1 = false;
    }

    public void SwitchAndWait(){
        if(status1){
            Open();
        }
        else{
            Close();
        }
    }

}
