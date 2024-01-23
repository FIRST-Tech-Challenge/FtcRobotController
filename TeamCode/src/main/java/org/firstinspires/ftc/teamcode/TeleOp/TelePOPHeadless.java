package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Inter;

import java.io.File;

@TeleOp(name="TelePOPHeadless", group="TelePOP")
//@Disabled
public class TelePOPHeadless extends LinearOpMode implements Inter {

    //Железо
    private DcMotor LeftFront3, LeftBack2, RightFront0, RightBack1, motoOnTele1, Capture0, led;
    public DistanceSensor r1, r2;
    private Servo Plain, UnderTele, Hook;
    private BNO055IMU imu;
    private DigitalChannel touch;

    //Переменные моторов
    private double zm1, zm2, zm3, zm4, zm5, zm6, zm7, zm8, zLED;
    private double zs1 = 0.74;

    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0, last_moment_auto_down = 0.0, last_moment_auto_sides = 0.0, last_moment_auto_up = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free, moment_diff_auto_down, moment_diff_auto_sides, moment_diff_auto_up;
    private boolean auto_mode = true, free_mode = false;
    private double a, a_telescope, vyr, turn, alpha, beta, theta, sina, cosa, sinb, cosb, sint, cost, x, y;
    private int strela_req_level;
    private double strela_level;
    private Orientation angles;
    private String teleservo = "неактивен";
    private String telemode = "автозахват";
    private String telestable = "стабилен";
    private String telespeed = "стабильная";
    private String pressed = "не нажат";
    private boolean pos_servoscop = false;
    private boolean last_press_servoscop = false;
    private float dgr = 0;
    double LastAngle = 0;
    int telescopePos = 0;
    int rot_vyr;
    private double volt = 0;
    private boolean reinit = false;
    private boolean reverse = false;
    private boolean auto = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double threshc = 0.02;

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с отклонением угла в конце автономки от начального угла

    //Гироскоп
    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Инициализируем железо
    public void initC() {
        //Инициализация
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

        initIMU();

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

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
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

                    LastAngle = Double.parseDouble(ReadWriteFile.readFile(angleFile).trim()); //Отклонение угла от начального

                    while (!isStopRequested() & opModeIsActive()) {

                        //ТЕЛЕЖКА

                        //Угол выравнивания
                        LastAngle = !reinit ? LastAngle : 0;

                        //Выравнивание тележки
                        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                        if (gamepad1.dpad_up == true) {
                            vyr = ((angles.firstAngle + LastAngle) / 25);
                        }
                        else if (gamepad1.dpad_right == true) {
                            vyr = ((angles.firstAngle + LastAngle + 90) / 25);
                        }
                        else if (gamepad1.dpad_down == true) {
                            if (angles.firstAngle > 0) {
                                vyr = ((angles.firstAngle + LastAngle - 180) / 25);
                            }
                            if (angles.firstAngle < 0) {
                                vyr = ((angles.firstAngle + LastAngle + 180) / 25);
                            }
                        }
                        else if (gamepad1.dpad_left == true) {
                            vyr = ((angles.firstAngle + LastAngle - 90) / 25);
                        }
                        else {
                            vyr = 0;
                        }

                        //Коэффицент скорости робота
                        a = gamepad1.left_trigger < 0.15 ? 0.5 : 1;

                        //Поворот
                        turn = gamepad1.right_stick_x;

                        theta = angles.firstAngle;
                        beta = theta + 90;

                        sinb = Math.sin(Math.toRadians(beta));
                        cosb = Math.cos(Math.toRadians(beta));

                        sint = Math.sin(Math.toRadians(theta));
                        cost = Math.cos(Math.toRadians(theta));

                        x = gamepad1.left_stick_x;
                        y = gamepad1.left_stick_y;

                        //Мощность моторов тележки
                        zm1 = Range.clip(((-sinb - cosb)*y + (-sint - cost)*x - turn - vyr) * a, -1, 1);
                        if (zm1 > -0.05 && zm1 < 0.05) {
                            zm1 = 0;
                        }

                        zm2 = Range.clip(((-sinb + cosb)*y + (-sint + cost)*x - turn - vyr) * a, -1, 1);
                        if (zm2 > -0.05 && zm2 < 0.05) {
                            zm2 = 0;
                        }

                        zm3 = Range.clip(((sinb + cosb)*y + (sint + cost)*x - turn - vyr) * a, -1, 1);
                        if (zm3 > -0.05 && zm3 < 0.05) {
                            zm3 = 0;
                        }

                        zm4 = Range.clip(((sinb - cosb)*y + (sint - cost)*x - turn - vyr) * a, -1, 1);
                        if (zm4 > -0.05 && zm4 < 0.05) {
                            zm4 = 0;
                        }

                        //Мощность телескопа
                        if (gamepad2.left_stick_y < 0.05) {
                            a_telescope = 0.7;
                        } else if (gamepad2.left_stick_y > 0.05) {
                            if (motoOnTele1.getCurrentPosition() < 175) {
                                a_telescope = 0.4;
                            } else {
                                a_telescope = 0.125;
                            }
                        } else {
                            a_telescope = 0;
                        }

                        zm5 = Range.clip(gamepad2.left_stick_y * a_telescope, -1, 1);

                        //Захват конусов
                        moment_diff_serv = runtime.milliseconds() - last_moment_serv;
                        moment_diff_switch = runtime.milliseconds() - last_moment_switch;

                        //Переключение режимов Автоматический-Ручной
                        if (gamepad2.back == true && moment_diff_switch > 200) {
                            if (auto_mode == false) {
                                auto_mode = true;
                            } else {
                                auto_mode = false;
                            }
                            last_moment_switch = runtime.milliseconds();
                        }

                        //Автоматический захват
                        if (auto_mode == true && touch.getState() == false && pos_servoscop == false && moment_diff_serv >= 500) {
                            zs1 = 0.66;
                            pos_servoscop = true;
                            last_moment_serv = runtime.milliseconds();
                        }

                        //Ручной захват
                        if (gamepad2.x == true && moment_diff_serv > 200) {
                            if (pos_servoscop == false) {
                                zs1 = 0.66;
                                pos_servoscop = true;
                            } else {
                                zs1 = 0.74;
                                pos_servoscop = false;
                            }
                            last_moment_serv = runtime.milliseconds();
                        }

                        //Свечение
                        if (teleservo == "неактивен") {
                            zLED = Math.abs(0);
                        }
                        if (teleservo == "активен") {
                            zLED = Math.abs(0.025);
                        }

                        //Состояние скорости робота
                        if (a == 0.5) {
                            telespeed = "стабильная";
                        } else {
                            telespeed = "форсаж";
                        }

                        //Состояние захвата
                        if (auto_mode == false) {
                            telemode = "ручной";
                        } else {
                            telemode = "автозахват";
                        }

                        //Состояние ограничений
                        if (free_mode == false) {
                            telestable = "стабилен";
                        } else {
                            telestable = "без ограничений";
                        }

                        //Состояние захвата
                        if (pos_servoscop == false) {
                            teleservo = "неактивен";
                            zs1 = 0.66;

                        } else {
                            teleservo = "активен";
                            zs1 = 0.74;

                        }

                        //Нажатие на датчик касания
                        if (touch.getState() == true) {
                            pressed = "не нажат";
                        } else {
                            pressed = "нажат";
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

            if (gamepad1.y) { initIMU(); reinit = true; } //Обнуление гироскопа

            telemetry.addData("Состояние захвата", teleservo);
            telemetry.addData("Энкодер телескопа", motoOnTele1.getCurrentPosition());
            telemetry.addData("Датчик захвата", pressed);
            telemetry.addData("Режим захвата", telemode);
            telemetry.addData("Режим", telestable);
            telemetry.addData("Скорость", telespeed);
            telemetry.addData("Мотор слева спереди", LeftBack2.getCurrentPosition());
            telemetry.addData("Мотор слева сзади", LeftFront3.getCurrentPosition());
            telemetry.addData("Мотор справа спереди", RightFront0.getCurrentPosition());
            telemetry.addData("Мотор справа сзади", RightBack1.getCurrentPosition());
            telemetry.addData("Угол", angles.firstAngle);
            telemetry.update();

        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, Double.toString(angles.firstAngle));
    }
}