/* La inceputul programului, exista import-urile. Ele se fac de obicei automat asa ca nu te ingrijora de ele numai daca dau eroare(Nu au dat niciodata lol).
De asemenea, daca ceva da eroare in cod si nu stii de ce, verifica mai intai daca este importata chestia sau nu.
 */
package org.firstinspires.ftc.teamcode;
import static java.lang.Math.abs;
import static java.lang.Math.addExact;
import static java.lang.Math.floorDiv;

import android.util.Log;
import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class centerstageRX extends OpMode {
    public Switch swish;
    public DcMotorEx motorBR,motorBL,motorFL,motorFR,sliderL,sliderR,rotitor_sus,rotitor_jos;
    public Servo gherutaL,gherutaR,plauncher;
    public CRServo maceta;
    double sm = 1, slow = 1, lb = 1, rb = 1,bval=0,xval=0;
    boolean dpd,dpu,enter = false,bl,xl,aprins1 = false,aprins2 = false;
    double y, x, rx, ghearaPoz=0.5, macetaPow=0;
    double max = 0,lastTime;
    double pmotorBL;
    double pmotorBR;
    double pmotorFL;
    double pmotorFR;

    boolean stop = false, lastx = false, lasty = false, sliderState = true, aIntrat = false,aAjuns = true,aInchis = true;
    double intPoz = 0.4, servoPos = 0.0;
    /*Functia de init se ruleaza numai o data, se folosete pentru initializarea motoarelor si chestii :)*/
    @Override
    public void init() {
        /* Liniile astea de cod fac ca motoarele sa corespunda cu cele din configuratie, cu numele dintre ghilimele.*/
        motorBL = hardwareMap.get(DcMotorEx.class, "motorBL"); // Motor Back-Left
        motorBR = hardwareMap.get(DcMotorEx.class, "motorBR"); // Motor Back-Right
        motorFL = hardwareMap.get(DcMotorEx.class, "motorFL"); // Motor Front-Left
        motorFR = hardwareMap.get(DcMotorEx.class, "motorFR"); // Motor Front-Right
        sliderL = hardwareMap.get(DcMotorEx.class,"sliderL");
        sliderR = hardwareMap.get(DcMotorEx.class,"sliderR");
        rotitor_sus = hardwareMap.get(DcMotorEx.class,"rotitor_sus");
        rotitor_jos = hardwareMap.get(DcMotorEx.class,"rotitor_jos");

        gherutaL = hardwareMap.get(Servo.class,"gherutaL");
        gherutaR = hardwareMap.get(Servo.class,"gherutaR");
        maceta = hardwareMap.get(CRServo.class,"maceta");
        plauncher = hardwareMap.get(Servo.class,"plauncher");

        motorBL.setDirection(DcMotorEx.Direction.REVERSE);
        motorFL.setDirection(DcMotorEx.Direction.REVERSE);
        sliderL.setDirection(DcMotorEx.Direction.REVERSE);
        rotitor_sus.setDirection(DcMotorEx.Direction.REVERSE);

        motorBL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sliderL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        sliderR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotitor_sus.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rotitor_jos.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        rotitor_jos.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotitor_sus.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motorFR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        sliderL.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sliderR.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotitor_jos.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rotitor_sus.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void start(){
        Chassis.start();
        Systems.start();
    }
    /*Aici se declara thread-ul cu numele chassis, pentru ca contine partea de program care se ocupa de sasiu*/
    private final Thread Chassis = new Thread(new Runnable() {
        @Override
        public void run(){
            /*Thread-urile nu vor rula la infinit fara acest while, ci vor rula numai o data. Asta este foarte folositor pentru Telecomandat, dar fara while se pot face thread-uri pentru autonom in unele cazuri*/
            while(!stop) {
                /* Liniile astea de cod iau input-ul controller-ului si il pun in niste variabile*/
                y = gamepad1.left_stick_y;
                x = gamepad1.left_stick_x;
                rx = gamepad1.right_stick_x;

                /* Liniile astea de cod iau niste variabile care reprezinta puterea fiecarui motor, cu ajutorul puterilor de la controller*/
                pmotorFL = -y + x + rx;
                pmotorBL = -y - x + rx;
                pmotorBR = -y + x - rx;
                pmotorFR = -y - x - rx;

                /*Secventele urmatoare de cod stabilesc maximul dintre modulele puterilor motoarelor cu un anumit scop...*/
                max = abs(pmotorFL);
                if (abs(pmotorFR) > max) {
                    max = abs(pmotorFR);
                }
                if (abs(pmotorBL) > max) {
                    max = abs(pmotorBL);
                }
                if (abs(pmotorBR) > max) {
                    max = abs(pmotorBR);
                }
                /*...care este punerea tuturor puterilor motoarelor sub 1, cum puterile de la motoare pot fi numai intre 1 si -1*/
                if (max > 1) {
                    pmotorFL /= max;
                    pmotorFR /= max;
                    pmotorBL /= max;
                    pmotorBR /= max;
                }
                if(gamepad1.x != lastx){
                    rb += 0.5;
                    if(rb > 2){
                        rb = 0.5;
                    }
                }
                if(gamepad1.y != lasty){
                    lb += 0.5;
                    if(lb > 2){
                        lb = 0.5;
                    }
                }
                if(rb == 2){
                    sm = 4;
                }
                else if(lb == 2){
                    sm = 2;
                }
                else{
                    sm = 1;
                }
                lastx = gamepad1.x;
                lasty = gamepad1.y;
                POWER(pmotorFR / sm, pmotorFL / sm, pmotorBR / sm, pmotorBL / sm);
            }
        }
        public void POWER(double df1, double sf1, double ds1, double ss1){
            motorFR.setPower(df1);;
            motorBL.setPower(ss1);
            motorFL.setPower(sf1);
            motorBR.setPower(ds1);
        }
    });
    /*Aici se declara thread-ul cu numele systems, pentru ca contine partea de program care se ocupa de sisteme*/
    private final Thread Systems = new Thread(new Runnable() {
        @Override
        public void run() {
            while (!stop) {
//                if(xl != gamepad2.x){
//                    xval += 0.5;
//                    if(xval >= 2){
//                        xval = 0;
//                    }
//                }
//                xl = gamepad2.x;
//                if(xval==1) {
//                    if(gamepad2.dpad_down){
//                        maceta.setPower(1);
//                    }
//                    if(gamepad2.dpad_up){
//                        maceta.setPower(-1);
//                    }
//                }
//                if(xval!=1){
//                    maceta.setPower(0);
//                }
                    if(gamepad2.dpad_down && maceta.getPower() == 0){
                        maceta.setPower(1);
                    }
                    if(gamepad2.dpad_up && maceta.getPower() == 0){
                        maceta.setPower(-1);
                    }
                    if ((gamepad2.dpad_up || gamepad2.dpad_down) && maceta.getPower() != 0){
                        maceta.setPower(0);
                    }


                dpd = gamepad2.dpad_down;
                dpu = gamepad2.dpad_up;
                sliderL.setPower(gamepad2.right_stick_y);
                sliderR.setPower(gamepad2.right_stick_y);
                if(gamepad2.left_trigger > 0){
                    slow = 2;
                }
                else{
                    slow = 1;
                }
                if(gamepad2.right_trigger > 0){
                    plauncher.setPosition(0.5);
                }
                else{
                    plauncher.setPosition(0);
                }
                rotitor_jos.setPower(gamepad2.left_stick_y/slow);
                rotitor_sus.setPower(gamepad2.left_stick_y/slow);
                if(gamepad2.b != bl){
                    bval+=0.5;
                    if(bval>=1){
                        bval=0;
                    }
                }
                bl = gamepad2.b;
                if(bval==0.5 && gamepad2.b && !enter) {
                    enter = true;
                    gherutaR.setPosition(0.15);
                    gherutaL.setPosition(0.63);
                    lastTime = System.currentTimeMillis();
                    while (lastTime + 100 > System.currentTimeMillis()) {
                    }
                    gherutaL.setPosition(0.38);
                    gherutaR.setPosition(0.38);
                    enter = false;
                }
                if (gamepad2.a){
                    gherutaR.setPosition(0.15);
                    gherutaL.setPosition(0.63);
                }
                if (gamepad2.y){
                    gherutaR.setPosition(0.38);
                    gherutaL.setPosition(0.38);
                }
            }
        }
    });
    /*Aici se afla partea de program care arata cand programul se opreste, este foarte folositor pentru functionarea thread-urilor*/
    public void stop(){stop = true;}

    /*Aici se afla partea de telemetrie a robotului.
    Telemetria iti arata pe driver hub/telefon cu driver station o valoare pe care ai stabilit-o, cu un anumit text dinaintea lui*/
    @Override

    public void loop() {
        telemetry.addData("motorBL", motorBL.getCurrentPosition());
        telemetry.addData("motorFL", motorFL.getCurrentPosition());
        telemetry.addData("motorBR", motorBR.getCurrentPosition());
        telemetry.addData("motorFR", motorFR.getCurrentPosition());
        telemetry.addData("sliderL:", sliderL.getCurrentPosition());
        telemetry.addData("sliderR:", sliderR.getCurrentPosition());
        telemetry.addData("rotitor_jos:", rotitor_jos.getCurrentPosition());
        telemetry.addData("rotitor_sus:", rotitor_sus.getCurrentPosition());
        telemetry.addData("ghearaPoz:", ghearaPoz);
        telemetry.addData("macetaPow:", macetaPow);
        telemetry.addData("bval:", bval);
        telemetry.addData("xval:", xval);
        telemetry.addData("bl:", bl);
        telemetry.addData("gamepad2.b:", gamepad2.b);
    }
}