package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Inter;

import java.io.File;
import java.util.Timer;

@TeleOp(name="TelePOP", group="Lagrange")
//@Disabled
public class TelePOP extends LinearOpMode implements Inter {
    //Таймер
    Timer time = new Timer();
    //Железо
    private DcMotor LeftFront3, LeftBack2, RightFront0, RightBack1, motoOnTele1,Capture0;
    private Servo Plain, UnderTele, Hook;
    private DigitalChannel touch;

    //Переменные моторов

    private double test=0;
    private double zm1, zm2, zm3, zm4, zm5;
    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free;
    private boolean auto_mode = true, free_mode = false;
    private double a, turn;
    int telescopePos = 0;
    private ElapsedTime runtime = new ElapsedTime();
    private double lamp=0;
    private int height;
    File telescopeFile = AppUtil.getInstance().getSettingsFile("telescopeFile.txt"); //Файл с позицией телескопа
    private int svob=0;
    //Гироскоп

    //Инициализируем железо
    public void initC() {
        //Инициализация
        LeftFront3 = hardwareMap.get(DcMotor.class, "LeftFront3");
        LeftBack2 = hardwareMap.get(DcMotor.class, "LeftBack2");
        RightFront0 = hardwareMap.get(DcMotor.class, "RightFront0");
        RightBack1 = hardwareMap.get(DcMotor.class, "RightBack1");
        motoOnTele1 = hardwareMap.get(DcMotor.class, "motoOnTele1");
        Capture0 = hardwareMap.get(DcMotor.class, "Capture0");

        Plain = hardwareMap.get(Servo.class, "Plain");
        UnderTele = hardwareMap.get(Servo.class, "UnderTele");
        Hook = hardwareMap.get(Servo.class, "Hook");

        LeftFront3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LeftBack2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightFront0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightBack1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoOnTele1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Capture0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftFront3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftBack2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightBack1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoOnTele1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Capture0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftFront3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftBack2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightFront0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightBack1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motoOnTele1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Capture0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    LeftFront3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    LeftBack2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightFront0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    RightBack1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motoOnTele1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Capture0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {


                        //ТЕЛЕЖКА


                        //Коэффицент скорости робота
                        if(gamepad1.left_trigger<0.5){
                            a=0.7/1;
                        }
                        else if(gamepad1.left_trigger>0.5){
                            a=10/1;
                        }
//                        if(gamepad2.right_stick_y <0.5){
//                            m5.setPower(gamepad2.right_stick_y);
//                        }
                        //Поворот
                        turn = -gamepad1.right_stick_x;


                        //Мощность моторов
                        zm1 = Range.clip((-gamepad1.left_stick_x + gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip((-gamepad1.left_stick_x - gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = Range.clip((gamepad1.left_stick_x - gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip((gamepad1.left_stick_x + gamepad1.left_stick_y - turn ) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }

                        //ТЕЛЕСКОП

                        if(gamepad1.left_bumper==true){

                            if(gamepad1.left_trigger > 0.08){
                                motoOnTele1.setPower(gamepad1.left_trigger*100);
                            }

                            if(gamepad1.right_trigger>0.08){
                                motoOnTele1.setPower(gamepad1.right_trigger*-100);
                            }

                        }
                        else {
                            if(gamepad1.left_trigger > 0.08){
                                motoOnTele1.setPower(gamepad1.left_trigger*1);
                            }
                            if(gamepad1.right_trigger>0.08){
                                motoOnTele1.setPower(gamepad1.right_trigger*-1);
                            }
                        }
                        //Захват колёсами
                        if (gamepad1.right_bumper == true){
                            Capture0.setPower(-1);
                        }else {Capture0.setPower(0);}

                        //Захват конусов
                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                        //Ручной захват
//                        if(touch.getState() == false){
//                            zs5 = CLOSE;
//                            lamp = -0.1;
//                            moment_diff_serv =205;
//                        }
//
//                     if(gamepad1.a==true && moment_diff_serv > 200) {
//                         if (zs5 == CLOSE) {
//                             zs5 = OPEN;
//                             lamp = 0;
//                             last_moment_serv = runtime.milliseconds();
//                         }else{
//                                 zs5 = CLOSE;
//                                 lamp = -0.1;
//                             last_moment_serv = runtime.milliseconds();
//                             }
//
////                         } else {
////
////
////                             while (touch.getState() == true && m5.getCurrentPosition() >= 10) {
////                                 m5.setPower(0.7);
////                             }
////                             if (touch.getState() == false) {
////                                 zs5 = CLOSE;
////                                 lamp = -0.1;
////                             }
////
////
////
////                         }
////                         last_moment_serv = runtime.milliseconds();
//                     }



//                        if (gamepad2.a == true && moment_diff_serv > 350) {
//                            if (zs5 == CLOSE) {
//                                zs5 = OPEN;
//                                lamp=0;
//                            } else {
//                                zs5 = CLOSE;
//                                lamp=-0.1;
//                            }
//                            last_moment_serv = runtime.milliseconds();
//                        }



//                        if(svob==0) {
//                            if (gamepad2.a == true && moment_diff_serv > 350) {
//                                if (zs5 == CLOSE) {
//                                    zs5 = OPEN;
//                                    lamp=0;
//                                } else {
//                                    zs5 = CLOSE;
//                                lamp=-0.1;
//                                }
//                                last_moment_serv = runtime.milliseconds();
//                            }
//                        }
//                        //авто захват
//                        if (svob==1 && gamepad2.x == true) {
//
//                           while (touch.getState() == true || m5.getCurrentPosition() == 0 ){
//                               m5.setPower(0.8);
//                           }
//                            if (touch.getState() == false) {
//                                zs5 = CLOSE;
//                            }
//
//                            if (gamepad2.a == true) {
//                                zs5 = OPEN;
//                            }
//                        }

                        //Переключение режимов Автоматический-Ручной
                        if (gamepad1.back == true && moment_diff_switch > 350) {
                            if (auto_mode == false) {
                                auto_mode = true;
                            } else {
                                auto_mode = false;
                            }
                            last_moment_switch = runtime.milliseconds();
                        }



                    }

                } catch (Exception e) {
                    telemetry.addLine("Calc thread interrupted");
                    telemetry.update();
                }
            }
            public void start_c() {
                if (c == null) {
                    c = new Thread(this, "Calc thread");
                    c.start();
                }
            }
        }

        //Инициализация
        initC();

        waitForStart();

        //Запуск подпроцессов
        CalcThread C1 = new CalcThread();
        C1.start_c();

        //ОСНОВНАЯ ПРОГРАММА

        while(opModeIsActive() & !isStopRequested()) {

            LeftFront3.setPower(zm1);//слева спереди
            RightFront0.setPower(zm2);//справа спереди
            LeftBack2.setPower(zm3);//слева сзади
            RightBack1.setPower(zm4);//справа сздади
            motoOnTele1.setPower(zm5);//барабан

            Plain.setPosition(OPEN);
            Hook.setPosition(OPEN);
            UnderTele.setPosition(OPEN);


            telemetry.addData("Состояние тригера", gamepad1.left_trigger);
            telemetry.addData("коэфицент скорости", a);
            telemetry.addData("Svod", svob);
            telemetry.addData("Мотор1", zm1);
            telemetry.addData("Мотор2", zm2);
            telemetry.addData("Мотор3", zm3);
            telemetry.addData("Мотор4", zm4);
            telemetry.addData("Мотор5", zm5);
            telemetry.addData("Стик1 X", gamepad1.left_stick_x);
            telemetry.addData("Стик1 Y", gamepad1.left_stick_y);
            telemetry.addData("Стик2 X", gamepad2.right_stick_x);
            telemetry.addData("Стик2 Y", gamepad2.right_stick_y);
            telemetry.addData("Уровень по энкодеру", motoOnTele1.getCurrentPosition());
            telemetry.addData("Ускорение", a);
            telemetry.update();

        };
        ReadWriteFile.writeFile(telescopeFile, Integer.toString(telescopePos));
    }
}