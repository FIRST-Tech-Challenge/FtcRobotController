package org.firstinspires.ftc.teamcode.Autonomus.methods;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Camera.Detector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;

@Disabled
public class AutoMethods extends LinearOpMode {

    public OpenCvCamera webcam;
    public boolean camError = false;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode op;
    public int baza = 1;

    //Железо
    public DcMotor leftFront, leftRear, rightFront, rightRear, motoOnTele1,Capture0;
    public DistanceSensor r1, r2;
    public Servo Plain, UnderTele, Hook;
    private BNO055IMU imu;
    private DigitalChannel touch;

    //Переменные моторов
    private double zm1, zm2, zm3, zm4, zm5, zm6, zm7;
    private double zs1 = 0.71;

    private double last_moment_serv = 0.0, last_moment_switch = 0.0, last_moment_free = 0.0, last_moment_auto_down = 0.0, last_moment_auto_sides = 0.0, last_moment_auto_up = 0.0;
    private double moment_diff_serv, moment_diff_switch, moment_diff_free, moment_diff_auto_down, moment_diff_auto_sides, moment_diff_auto_up;
    private boolean auto_mode = true, free_mode = false;
    private double a, a_telescope, vyr, turn;
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
    private double voltage = 0, voltage_k;
    private boolean reinit = false;
    private boolean reverse = false;
    private boolean auto = false;
    private double threshc = 0.02;

    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с позицией робота

    @Override
    public void runOpMode() throws InterruptedException {

    }

    public void initIMU(OpMode op) {
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Инициализируем железо
    public void initC(OpMode op) {
        this.op = op;
        //Инициализация
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        motoOnTele1 = hardwareMap.get(DcMotor.class, "motoOnTele1");
        Capture0 = hardwareMap.get(DcMotor.class, "Capture0");

        Plain = hardwareMap.get(Servo.class, "Plain");
        UnderTele = hardwareMap.get(Servo.class, "UnderTele");
        Hook = hardwareMap.get(Servo.class, "Hook");

        initIMU(op);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoOnTele1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Capture0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motoOnTele1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Capture0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motoOnTele1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Capture0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        touch = op.hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
    }

    public void camStart(OpMode op) {
        try {
            this.op = op;
            int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.openCameraDevice();
            webcam.setPipeline(new Detector(op.telemetry));
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
        } catch (OpenCvCameraException e1) {camError = true;}
        catch (NullPointerException e2) {camError = true;}
    }

    public void camStop() {
        if (!camError) {webcam.stopStreaming();}
    }

    public void getPos() {
        if (!camError) {
            switch (Detector.getLocation()) {
                case BLUE:
                    baza = 1;
                    break;
                case YELLOW:
                    baza = 2;
                    break;
                case STRIPES:
                    baza = 3;
                    break;
            }
        }
        camStop();
    }

    //Запись угла в файл
    public void writeAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, String.valueOf(angles.firstAngle));
    }

    public void nazad(OpMode op, double spd, double l, double timeout, double ang) {


//        double curr_encY = m2.getCurrentPosition();
//        double rasst = curr_encY - m2.getCurrentPosition();
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double vyr_znak;
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (rasst < l) && (runtime.milliseconds() - timeout * 1000) < 0) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = angles.firstAngle / 30;
//            vyr_znak = vyr/Math.abs(vyr);
//            vyr = vyr_znak * Range.clip(Math.abs(vyr), 0.02, 1);
//
//            m1.setPower(-spd - vyr);
//            m2.setPower(-spd - vyr);
//            m3.setPower(spd - vyr);
//            m4.setPower(spd - vyr);
//
//            rasst = curr_encY - m2.getCurrentPosition();
//
//            op.telemetry.addData("rasst", rasst);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(350);
    }

    public void vpered(OpMode op, double spd, double l, double timeout, double ang) {

//        double curr_encY = m2.getCurrentPosition();
//        double rasst = m2.getCurrentPosition() - curr_encY;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double vyr_znak;
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (rasst < l) && (runtime.milliseconds() - timeout * 1000) < 0) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = angles.firstAngle / 30;
//            vyr_znak = vyr/Math.abs(vyr);
//            vyr = vyr_znak * Range.clip(Math.abs(vyr), 0.02, 1);
//
//            m1.setPower(spd - vyr);
//            m2.setPower(spd - vyr);
//            m3.setPower(-spd - vyr);
//            m4.setPower(-spd - vyr);
//
//            rasst = m2.getCurrentPosition() - curr_encY;
//
//            op.telemetry.addData("rasst", rasst);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(350);
    }

    public void vpravo(OpMode op, double spd, double l, double timeout, double ang) {
//        double curr_encX = m1.getCurrentPosition();
//        double rasst = m1.getCurrentPosition() - curr_encX;
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double vyr_znak;
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (rasst < l) && (runtime.milliseconds() - timeout * 1000) < 0) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = angles.firstAngle / 30;
//            vyr_znak = vyr/Math.abs(vyr);
//            vyr = vyr_znak * Range.clip(Math.abs(vyr), 0.02, 1);
//
//            m1.setPower(-spd - vyr);
//            m2.setPower(spd - vyr);
//            m3.setPower(spd - vyr);
//            m4.setPower(-spd - vyr);
//
//            rasst = m1.getCurrentPosition() - curr_encX;
//
//            op.telemetry.addData("rasst", rasst);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(350);
    }

    public void vlevo(OpMode op, double spd, double l, double timeout, double ang) {
//        double curr_encX = m1.getCurrentPosition();
//        double rasst = curr_encX - m1.getCurrentPosition();
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double vyr_znak;
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (rasst < l) && (runtime.milliseconds() - timeout * 1000) < 0) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = angles.firstAngle / 30;
//            vyr_znak = vyr/Math.abs(vyr);
//            vyr = vyr_znak * Range.clip(Math.abs(vyr), 0.02, 1);
//
//            m1.setPower(spd - vyr);
//            m2.setPower(-spd - vyr);
//            m3.setPower(-spd - vyr);
//            m4.setPower(spd - vyr);
//
//            rasst = curr_encX - m1.getCurrentPosition();
//
//            op.telemetry.addData("rasst", rasst);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(350);
    }

    public void dvizh(OpMode op, double spd, double x, double y, double ang, double timeout, boolean reset) {
//        this.op = op;
//
//        double alpha;
//        double speed = spd;
//        double m1v, m2v, m3v, m4v;
//        double ugol, xv, yv, xc, yc, xl, yl, xp, yp, hypl, hyp, hypp;
//        double xl_znak, yl_znak, xl_znak_new, yl_znak_new;
//        boolean slowmode = false;
//
//        if (reset) {
//            m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
//
//        xc = m1.getCurrentPosition();
//        yc = m2.getCurrentPosition();
//
//        xl = x-xc;
//        yl = y-yc;
//
//        xl_znak = xl/Math.abs(xl);
//        yl_znak = yl/Math.abs(yl);
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && (Math.abs(xl) > 1 || Math.abs(yl) > 1) && (runtime.milliseconds() - timeout * 1000) < 0) {
//
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            ugol = 360 + angles.firstAngle;
//
//            alpha = 0;
//
//            xl = x-xc;
//            yl = y-yc;
//
//            if (yl > 0) {
//                if (xl >= 0) {
//                    alpha = Math.atan(xl/yl);
//                }
//                if (xl < 0) {
//                    alpha = 3.141592653589*2 - Math.abs(Math.atan(xl/yl));
//                }
//            }
//
//            if (yl < 0) {
//                if (x >= 0) {
//                    alpha = 3.141592653589 - Math.abs(Math.atan(xl/yl));
//                }
//                if (xl < 0) {
//                    alpha = 3.141592653589 + Math.atan(xl/yl);
//                }
//            }
//
//            if (yl == 0) {
//                if (xl >= 0) {
//                    alpha = 3.141592653589/2;
//                }
//                if (xl < 0) {
//                    alpha = -(3.141592653589/2);
//                }
//            }
//
//            xv = Math.sin(alpha);
//            yv = Math.cos(alpha);
//
//            m1v = yv - xv;
//            m2v = xv + yv;
//            m3v = xv - yv;
//            m4v = -xv - yv;
//
//            xc = m1.getCurrentPosition();
//            yc = m2.getCurrentPosition();
//
//            vyr = (angles.firstAngle + ang) / 23;
//
//            xp = (x-Math.abs(xl))/x;
//            yp = (y-Math.abs(yl))/y;
//
//            hyp = Math.sqrt(x*x + y*y);
//            hypl = Math.sqrt(xl*xl + yl*yl);
//            hypp = (hyp-hypl)/hyp;
//
//            xl_znak_new = xl/Math.abs(xl);
//            yl_znak_new = yl/Math.abs(yl);
//
//            if (hypl <= 180) {
//                slowmode = true;
//            }
//            if (slowmode == true) {
//                spd = Range.clip(
//                        ((0.22 + hypl/270)),
//                        0.2,
//                        0.5);
//            }
//            if (slowmode == false) {
//                spd = speed;
//            }
//
//            voltage = op.hardwareMap.getAll(VoltageSensor.class).iterator().next().getVoltage();
//            voltage_k = (12.5/voltage);
//
//            m1.setPower(Range.clip((m1v - vyr), -1, 1) * spd * voltage_k);
//            m2.setPower(Range.clip((m2v - vyr), -1, 1) * spd * voltage_k);
//            m3.setPower(Range.clip((m3v - vyr), -1, 1) * spd * voltage_k);
//            m4.setPower(Range.clip((m4v - vyr), -1, 1) * spd * voltage_k);
//
//            //op.telemetry.addData("Угол альфа (в градусах)", Math.toDegrees(alpha));
//            //op.telemetry.addData("Угол альфа (в радианах)", alpha);
//            op.telemetry.addData("Энкодер оси x", m1.getCurrentPosition());
//            op.telemetry.addData("Энкодер оси y", m2.getCurrentPosition());
//            op.telemetry.addData("Вольтаж", voltage);
//            op.telemetry.addData("Скорость", spd);
//            op.telemetry.addData("hypl", hypl);
//            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(350);
    }

    public void dvizh_passive() {

//        double x = 0;
//        double y = 0;
//        double speed = 0;
//
//        double alpha, m1v, m2v, m3v, m4v, ugol, xv, yv, xc, yc, xl, yl;
//        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//
//        xc = m1.getCurrentPosition();
//        yc = m2.getCurrentPosition();
//
//        xl = x-xc;
//        yl = y-yc;
//
//        while (opModeIsActive() && !isStopRequested()) {
//
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            ugol = 360 + angles.firstAngle;
//
//            alpha = 0;
//
//            xl = x-xc;
//            yl = y-yc;
//
//            if (yl > 0) {
//                if (xl >= 0) {
//                    alpha = Math.atan(xl/yl);
//                }
//                if (xl < 0) {
//                    alpha = 3.141592653589*2 - Math.abs(Math.atan(xl/yl));
//                }
//            }
//
//            if (yl < 0) {
//                if (x >= 0) {
//                    alpha = 3.141592653589 - Math.abs(Math.atan(xl/yl));
//                }
//                if (xl < 0) {
//                    alpha = 3.141592653589 + Math.atan(xl/yl);
//                }
//            }
//
//            if (yl == 0) {
//                if (xl >= 0) {
//                    alpha = 3.141592653589/2;
//                }
//                if (xl < 0) {
//                    alpha = -(3.141592653589/2);
//                }
//            }
//
//            xv = Math.sin(alpha);
//            yv = Math.cos(alpha);
//
//            m1v = yv - xv;
//            m2v = xv + yv;
//            m3v = xv - yv;
//            m4v = -xv - yv;
//
//            xc = m1.getCurrentPosition();
//            yc = m2.getCurrentPosition();
//
//            vyr = angles.firstAngle / 18;
//
//            //op.telemetry.addData("Угол альфа (в градусах)", Math.toDegrees(alpha));
//            //op.telemetry.addData("Угол альфа (в радианах)", alpha);
//            op.telemetry.addData("Энкодер оси x (1)", m2.getCurrentPosition());
//            op.telemetry.addData("Энкодер оси x (2)", m3.getCurrentPosition());
//            op.telemetry.addData("Энкодер оси y", m1.getCurrentPosition());
//            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
//            op.telemetry.addData("Угол робота", angles.firstAngle);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
//
//        sleep(100);
    }

    public double convert_ang(double angle, double angle_b) {
//        angle = angle - angle_b;
//        if (angle < -180) {
//            angle = 360 + angle;
//        }
//        if (angle > 180) {
//            angle = -360 + angle;
//        }
        return angle;
    }

    public void rotate(double ugolok, double timeout) {
//
//        double vyr_znak;
//
//        ugolok = -ugolok;
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        runtime.reset();
//
//        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > 0.25 && (runtime.milliseconds() - timeout * 1000) < 0) {
//
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = convert_ang(angles.firstAngle, ugolok) / 30;
//            vyr_znak = vyr/Math.abs(vyr);
//            vyr = vyr_znak * Range.clip(Math.abs(vyr), 0.02, 1);
//
//            m1.setPower(-vyr);
//            m2.setPower(-vyr);
//            m3.setPower(-vyr);
//            m4.setPower(-vyr);
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
    }

    public void tele(double pos) {
//
//        boolean lower = false;
//        boolean higher = false;
//
//        if (m5.getCurrentPosition() < pos) {
//            lower = true;
//        }
//        if (m5.getCurrentPosition() > pos) {
//            higher = true;
//        }
//
//        if (m5.getCurrentPosition() < pos) {
//            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() < pos) {
//                m5.setPower(-0.8);
//            }
//        }
//
//        if (m5.getCurrentPosition() > pos) {
//            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() > pos) {
//                m5.setPower(0.3);
//            }
//        }
//
//        if (lower) {
//            m5.setPower(-0.36);
//        }
//        if (higher) {
//            m5.setPower(0);
//        }
//        if (pos > 175) {
//            m5.setPower(-0.19);
//        }
    }

    public void rotate_zamer() {
//        int vyr_znak;
//        double ugolok = 90;
//
//        ugolok = -ugolok;
//
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > -1) {
//
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//            vyr = (angles.firstAngle - ugolok) / 150;
//            vyr_znak = (int) (vyr/Math.abs(vyr));
//
//            m1.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
//            m2.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
//            m3.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
//            m4.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
//
//            op.telemetry.addData("Энкодер оси x", m1.getCurrentPosition());
//            op.telemetry.addData("Энкодер оси y", m2.getCurrentPosition());
//            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
//            op.telemetry.addData("Угол робота", angles.firstAngle);
//            op.telemetry.update();
//        }
//
//        m1.setPower(0);
//        m2.setPower(0);
//        m3.setPower(0);
//        m4.setPower(0);
    }
}
