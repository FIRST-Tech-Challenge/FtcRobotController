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

    private Servo _20kg, povorot, kleshni, korzina;

    private ElapsedTime runtime = new ElapsedTime();

    //Переменные моторов;
    private double rightFz1, rightBz2, leftBz3, leftFz4, teleskopeZ5;

    private boolean switchA, switchB, switchY, switchX, switchDPadUp;
    private boolean isKorzinaOpen, isPovorotTakingPos, isKleshniOpen, isdPadUp;

    int dpadUp = 1;
    private double accel;

    //Инициализируем железо
    public void initC() {
        //Инициализация
        rightF = hardwareMap.get(DcMotor.class, "rightF");//m1
        rightB = hardwareMap.get(DcMotor.class, "rightB");//m2
        leftB = hardwareMap.get(DcMotor.class, "leftB");//m3
        leftF = hardwareMap.get(DcMotor.class, "leftF");//m4

        teleskopUpStanding = hardwareMap.get(DcMotor.class, "teleskop");

        _20kg = hardwareMap.get(Servo.class, "20kg");
        povorot = hardwareMap.get(Servo.class, "povorot");
        kleshni = hardwareMap.get(Servo.class, "kleshni");
        korzina = hardwareMap.get(Servo.class, "korzina");

        _20kg.setPosition(CLOSE_20KG_POS);
        povorot.setPosition(POVOROT_THROW_POS);
        kleshni.setPosition(KLESHNI_CLOSE_POS);
        korzina.setPosition(KORZINA_TAKING_POS);

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

            public void run() {
                telemetry.addLine("Calc thread running");
                telemetry.update();

                try {
                    rightF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    rightB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    leftF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

                        //Коэффицент скорости робота
                        if(gamepad1.left_trigger < 0.5){
                            accel = 0.6;
                        }
                        else if(gamepad1.left_trigger > 0.5){
                            accel = 10;
                        }

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

                        teleskopeZ5 = Range.clip(leftStickY2, -1,1);
                        if(teleskopeZ5 > -0.05 && teleskopeZ5 < 0.05){
                            teleskopeZ5 = 0;
                        }

                        if(gamepad2.a && !switchA && !gamepad2.start){
                            isPovorotTakingPos = !isPovorotTakingPos;
                            switchA = true;
                        }
                        if(!gamepad2.a && switchA){
                            switchA = false;
                        }

                        if(gamepad2.b && !switchB && !gamepad2.start){
                            isKleshniOpen = !isKleshniOpen;
                            switchB = true;
                        }
                        if(!gamepad2.b && switchB){
                            switchB = false;
                        }

                        if(gamepad2.y && !switchY && !gamepad2.start){
                            isKorzinaOpen = !isKorzinaOpen;
                            switchY = true;
                        }
                        if(!gamepad2.y && switchY){
                            switchY = false;
                        }

                        if(gamepad2.dpad_up && !switchDPadUp){
                            dpadUp = Range.clip(dpadUp + 1, 1, 5);
                            isdPadUp = !isdPadUp;
                            switchDPadUp = true;
                        }
                        if(!gamepad2.dpad_up && switchDPadUp){
                            switchDPadUp = false;
                        }

                        if(gamepad2.dpad_down){
                            dpadUp = 1;
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

            if(isPovorotTakingPos){
                povorot.setPosition(POVOROT_TAKING_POS);
            }else {
                povorot.setPosition(POVOROT_THROW_POS);
            }

            if(isKleshniOpen){
                kleshni.setPosition(KLESHNI_OPEN_POS);
            }else {
                kleshni.setPosition(KLESHNI_CLOSE_POS);
            }

            if(isKorzinaOpen){
                korzina.setPosition(KORZINA_TAKING_POS);
            }else {
                korzina.setPosition(KORZINA_THROW_POS);
            }

            if(dpadUp == 1){
                _20kg.setPosition(CLOSE_20KG_POS);
            } else if (dpadUp == 2) {
                _20kg.setPosition(CLOSE_20KG_POS - 0.1);
            }else if(dpadUp == 3){
                _20kg.setPosition(CLOSE_20KG_POS - 0.2);
            } else if (dpadUp == 4) {
                _20kg.setPosition(CLOSE_20KG_POS - 0.3);
            }else {
                _20kg.setPosition(OPEN_20KG_POS);
            }

            // Выводим значения в телеметрию
            telemetry.addData("Сервак _20kg позиция:", _20kg.getPosition());
            telemetry.addData("Сервак povorot позиция:", povorot.getPosition());
            telemetry.addData("Сервак kleshni позиция:", kleshni.getPosition());
            telemetry.addData("Сервак korzina позиция:", korzina.getPosition());

            telemetry.addData("Ускорение", accel);

            telemetry.update();
        };
    }
}