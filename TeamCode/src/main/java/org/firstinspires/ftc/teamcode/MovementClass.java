package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//Clasa separata pentru movement-ul robotului / motoarelor / servomotoare
//Comentarii pentru future self

public class MovementClass {

    //Declararea variabelor ce reprezinta motoarele cu ajutorul clasei DcMotor

    DcMotor fl,fr,bl,br;
    //Constructorul clasei MovementClass.java pentru initializarea motoarelor si aplicarea configuratiei Control Hub-ului

    public MovementClass(OpMode opMode) {

        //Initializarea motoarelor cu ajutorul 'hardwaremap.get' cu denumirile motoarelor din aplicatia ftc

        fl = opMode.hardwareMap.get(DcMotor.class, "motorFL");
        fr = opMode.hardwareMap.get(DcMotor.class, "motorFL");
        bl = opMode.hardwareMap.get(DcMotor.class, "motorFL");
        br = opMode.hardwareMap.get(DcMotor.class, "motorFL");

        //Setarea directiei motoarelor din partea dreapta a robotului din cauza constructiei sale

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
        bl.setPower(0);
    }

    public void forward(int a) {
        fl.setPower(a);
        fr.setPower(a);
        br.setPower(a);
        bl.setPower(a);
    }

    public void back(int a) {
        fl.setPower(-a);
        fr.setPower(-a);
        br.setPower(-a);
        bl.setPower(-a);
    }

    public void left(int a) {
        fl.setPower(-a);
        fr.setPower(a);
        br.setPower(a);
        bl.setPower(-a);
    }

    public void right(int a) {
        fl.setPower(a);
        fr.setPower(-a);
        br.setPower(-a);
        bl.setPower(a);
    }

    public void turnright(int a) {
        fl.setPower(a);
        fr.setPower(-a);
        br.setPower(-a);
        bl.setPower(a);
    }

    public void turnleft(int a) {
        fl.setPower(-a);
        fr.setPower(a);
        br.setPower(a);
        bl.setPower(-a);
    }

    public void diagonalleftfront(int a) {
        fr.setPower(a);
        bl.setPower(a);
    }

    public void diagonalleftback(int a) {
        fr.setPower(-a);
        bl.setPower(-a);
    }

    public void diagonalrightfront(int a) {
        fl.setPower(a);
        br.setPower(a);
    }

    public void diagonalrightback(int a) {
        fl.setPower(-a);
        br.setPower(-a);
    }
}
