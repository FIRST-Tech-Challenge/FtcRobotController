package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp (name="TeleOP", group="INFINITY")
public class TeleOpPop extends LinearOpMode implements ConstsForTeleskope {

    //Железо
    private DcMotor rightF, rightB, leftB, leftF, teleskopUpStanding;

    private Servo horizontal, povorot, kleshni, povorotVer, kleshniVer;

    private ElapsedTime runtime = new ElapsedTime();

    //Переменные моторов;
    private double rightFz1, rightBz2, leftBz3, leftFz4, teleskopeZ5;

    private boolean switchA = false, switchB = false, switchY = false, switchX = false, switchDPadUp = false, switchRBumper = false, switchLTrigger = false;
    private boolean isPovorotVerTaking = false, isPovorot = true, isKleshniOpen = false, isdPadUp = false, iskleshniVer = false, isRBumper = false, isTriggerL;

    int dpadUp = 2;
    private double accel;

    //Инициализируем железо
    public void initC() {
        //Инициализация
        rightF = hardwareMap.get(DcMotor.class, "rightF");//m1
        rightB = hardwareMap.get(DcMotor.class, "rightB");//m2
        leftB = hardwareMap.get(DcMotor.class, "leftB");//m3
        leftF = hardwareMap.get(DcMotor.class, "leftF");//m4

        teleskopUpStanding = hardwareMap.get(DcMotor.class, "teleskope");

        horizontal = hardwareMap.get(Servo.class, "horizontal");
        povorot = hardwareMap.get(Servo.class, "povorot");
        kleshni = hardwareMap.get(Servo.class, "kleshni");
        povorotVer = hardwareMap.get(Servo.class, "povorotVer");
        kleshniVer = hardwareMap.get(Servo.class, "kleshniVer");

        horizontal.setPosition(INITCLOSE_HORIZONTAL_POS);
        povorot.setPosition(POVOROT_TAKING_POS);
        kleshni.setPosition(KLESHNI_CLOSE_POS);
        povorotVer.setPosition(POVOROTVER_THROW_POS);
        kleshniVer.setPosition(KLESHNIVER_CLOSE_POS);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1 = hardwareMap.get(DcMotor.class, "En1");
        // En2 = hardwareMap.get(DcMotor.class, "En2");
        // En3 = hardwareMap.get(DcMotor.class, "En3");

        rightF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        teleskopUpStanding.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //En1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // En2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //En3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightF.setDirection(DcMotorSimple.Direction.FORWARD);
        rightB.setDirection(DcMotorSimple.Direction.FORWARD);
        leftF.setDirection(DcMotorSimple.Direction.REVERSE);
        leftB.setDirection(DcMotorSimple.Direction.REVERSE);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        // En1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //En2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // En3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
        //  En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runOpMode() {

        class CalcThread implements Runnable {
            private Thread c;
            private boolean running;
//
//            private boolean KleshPov_done = false;
//            private boolean pov_done = false;
//            private  boolean kleshni_done = false;
//            private  int timePOV = 0;
//            private  int timeKlesh = 0;
//            private  int timeKleshPov = 0;
//            private  int tick = 75000;


            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//                    povorotVer.setPosition(POVOROTVER_TAKING_POS);
//                    horizontal.setPosition(OPEN_HORIZONTAL_POS);

                    //ВЕРНУТЬСЯ / НЕ ЗАБЫТЬ!!!!
                    // En1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // En3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    while (!isStopRequested() & opModeIsActive()) {

                        double leftStickY1 = -gamepad1.left_stick_y;
                        double leftStickX1 = gamepad1.left_stick_x;
                        double turn = gamepad1.right_stick_x;

                        double leftStickY2 = -gamepad2.left_stick_y;

                        //ТЕЛЕЖКА
                        accel = 0.7;
                        //Коэффицент скорости робота
                        if(gamepad1.right_trigger > 0.05){
                            accel = 0.3;
                        }
                        if(gamepad1.left_trigger > 0.05){
                            accel = 5.5555555555555555555;
                        }

//                        if(gamepad2.x && !pov_done && timePOV > tick){
//                            pov_done = true;
//                            timePOV = 0;
//                        }
//
//
//                        if(gamepad2.x && pov_done && timePOV > tick){
//                            povorot.setPosition(POVOROT_TAKING_POS);
//                            pov_done = false;
//                            timePOV = 0;
//                        }
//                        timePOV++;
//
//
//                        if(gamepad2.y && !kleshni_done && timeKlesh > tick){
//                            kleshni.setPosition(KLESHNI_OPEN_POS);
//                            kleshni_done = true;
//                            timeKlesh = 0;
//
//                        }
//
//                        if(gamepad2.y && kleshni_done && timeKlesh > tick){
//                            kleshni.setPosition(KLESHNI_CLOSE_POS);
//                            kleshni_done = false;
//                            timeKlesh = 0;
//                        }
//                        timeKlesh++;
//
//
//
//                        if(gamepad2.a && !KleshPov_done && timeKleshPov > tick){
//                            kleshniVer.setPosition(KLESHNIVER_OPEN_POS);
//                            KleshPov_done = true;
//                            timeKleshPov = 0;
//                        }
//
//
//                        if(gamepad2.a && KleshPov_done && timeKleshPov > tick){
//                            kleshniVer.setPosition(KLESHNIVER_CLOSE_POS);
//                            KleshPov_done = false;
//                            timeKleshPov = 0;
//                        }
//                        timeKleshPov++;
//
//
//
//                        if(gamepad2.dpad_up){
//                            horizontal.setPosition(0.6);
//                        }
//
//                        if(gamepad2.dpad_down){
//                            horizontal.setPosition(0.93);
//                        }



                        //Мощность моторов
                        rightFz1 = (Range.clip((leftStickY1 - leftStickX1 - turn ) * accel, -1, 1));
                        if (rightFz1 > -0.05 && rightFz1 < 0.05) {
                            rightFz1 = 0;
                        }

                        rightBz2 = Range.clip((leftStickY1 + leftStickX1  - turn ) * accel, -1, 1);
                        if (rightBz2 > -0.05 && rightBz2 < 0.05) {
                            rightBz2 = 0;
                        }

                        leftBz3 = (Range.clip((leftStickY1 - leftStickX1 + turn ) * accel, -1, 1));
                        if (leftBz3 > -0.05 && leftBz3 < 0.05) {
                            leftBz3 = 0;
                        }

                        leftFz4 = Range.clip((leftStickY1 + leftStickX1 + turn ) * accel, -1, 1);
                        if (leftFz4 > -0.05 && leftFz4 < 0.05) {
                            leftFz4 = 0;
                        }

                        teleskopeZ5 = Range.clip(-leftStickY2, -1,1);
                        if(teleskopeZ5 > -0.05 && teleskopeZ5 < 0.05){
                            teleskopeZ5 = 0;
                        }

                        if(gamepad2.b && !switchB && !gamepad2.start){
                            isPovorot = !isPovorot;
                            switchB = true;
                        }
                        if(!gamepad2.b && switchB){
                            switchB = false;
                        }



                        if(gamepad2.x && !switchX && !gamepad2.start){
                            iskleshniVer = !iskleshniVer;
                            switchX = true;
                        }
                        if(!gamepad2.x && switchX){
                            switchX = false;
                        }


                        if(gamepad2.right_bumper && !switchRBumper){
                            isRBumper = !isRBumper;
                            switchRBumper = true;
                        }
                        if(!gamepad2.right_bumper && switchRBumper){
                            switchRBumper = false;
                        }

                        //A

                        if(gamepad2.a && !switchA && !gamepad2.start){
                            isKleshniOpen = !isKleshniOpen;
                            switchA = true;
                        }
                        if(!gamepad2.a && switchA){
                            switchA = false;
                        }

                        if(gamepad2.y && !switchY && !gamepad2.start) {
                            isPovorotVerTaking = !isPovorotVerTaking;
                            switchY = true;
                            SLEEP(500);

                        }
                        if(!gamepad2.y && switchY){
                            switchY = false;
                        }

                        if(gamepad2.dpad_up && !switchDPadUp){
                            dpadUp = Range.clip(dpadUp + 1, 2, 5);
                            isdPadUp = !isdPadUp;
                            switchDPadUp = true;
                        }
                        if(!gamepad2.dpad_up && switchDPadUp){
                            switchDPadUp = false;
                        }

                        if(gamepad2.dpad_down){
                            dpadUp = 2;
                        }

                        if(gamepad2.left_trigger > 0.05 && !switchLTrigger){
                            isTriggerL = !isTriggerL;
                            switchLTrigger = true;
                        }
                        if(gamepad2.left_trigger < 0.05 && switchLTrigger) {
                            switchLTrigger = false;
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

            rightF.setPower(rightFz1);
            rightB.setPower(rightBz2);
            leftB.setPower(leftBz3);
            leftF.setPower(leftFz4);

            teleskopUpStanding.setPower(teleskopeZ5);

            if(isPovorot){
                povorot.setPosition(POVOROT_TAKING_POS);
           }else {
                povorot.setPosition(POVOROT_THROW_POS);
            }

            if(isKleshniOpen){
                kleshni.setPosition(KLESHNI_OPEN_POS);
            }else {
                kleshni.setPosition(KLESHNI_CLOSE_POS);
            }

            if(iskleshniVer){
                kleshniVer.setPosition(KLESHNIVER_OPEN_POS);
            }else {
                kleshniVer.setPosition(KLESHNIVER_CLOSE_POS);
            }

            if(isPovorotVerTaking){
                povorotVer.setPosition(POVOROTVER_TAKING_POS);
            }else {
                povorotVer.setPosition(POVOROTVER_THROW_POS);
            }

            if(dpadUp == 1){
                horizontal.setPosition(INITCLOSE_HORIZONTAL_POS);
            }
            else if(dpadUp == 2){
                horizontal.setPosition(CLOSE_HORIZONTAL_POS);
//            } else if (dpadUp == 2) {
//                _20kg.setPosition(Range.clip(CLOSE_20KG_POS - 0.08, OPEN_20KG_POS,CLOSE_20KG_POS));
//            }else if(dpadUp == 3){
//                _20kg.setPosition(Range.clip(CLOSE_20KG_POS - 0.16, OPEN_20KG_POS,CLOSE_20KG_POS));
//            } else if (dpadUp == 4) {
//                _20kg.setPosition(Range.clip(CLOSE_20KG_POS - 0.34, OPEN_20KG_POS,CLOSE_20KG_POS));
            }
            else {
                horizontal.setPosition(OPEN_HORIZONTAL_POS);
            }
            if(isTriggerL){
                kleshni.setPosition(KLESHNI_CLOSE_POS);
                isKleshniOpen = false;
                povorot.setPosition(POVOROT_TAKING_POS);
                isPovorot = true;
                SLEEP(1800);
                horizontal.setPosition(INITCLOSE_HORIZONTAL_POS);
                dpadUp = 1;
                isTriggerL = false;
            }

            if(isRBumper){
                povorotVer.setPosition(POVOROTVER_TAKING_POS);
                kleshniVer.setPosition(KLESHNIVER_OPEN_POS);
                SLEEP(500);
                povorot.setPosition(POVOROT_THROW_POS);
                horizontal.setPosition(CLOSE_HORIZONTAL_POS);
                SLEEP(500);
                kleshniVer.setPosition(KLESHNIVER_CLOSE_POS);
                SLEEP(500);
                kleshni.setPosition(KLESHNI_OPEN_POS);
                sleep(1000);

                isKleshniOpen = true;
                isPovorotVerTaking = true;
                isPovorot = false;
                iskleshniVer = false;
                dpadUp = 2;
                isRBumper = false;
            }

            // Выводим значения в телеметрию
            telemetry.addData("Сервак horizontal позиция:", horizontal.getPosition());
            telemetry.addData("Сервак povorot позиция:", povorot.getPosition());
            telemetry.addData("Сервак kleshni позиция:", kleshni.getPosition());
            telemetry.addData("Сервак povorotVer позиция:", povorotVer.getPosition());
            telemetry.addData("Сервак kleshniVer позиция:", kleshniVer.getPosition());
            telemetry.addData("isTriggerL:", isTriggerL);

            telemetry.addData("Ускорение", accel);


            telemetry.update();
        };
    }
    public void SLEEP(double time){
        while (runtime.milliseconds() < time){

        }
        runtime.reset();
    }
}