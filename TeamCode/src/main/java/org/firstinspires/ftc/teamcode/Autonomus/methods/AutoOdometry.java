package org.firstinspires.ftc.teamcode.Autonomus.methods;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
public class AutoOdometry extends LinearOpMode {

    public OpenCvCamera webcam;
    public boolean camError = false;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode op;
    public int baza = 1;

    //Железо
    public DcMotor m1, m2, m3, m4, m5, m6, m7, led;
    public DistanceSensor r1, r2;
    public Servo s1, s2, s3, s4;
    private BNO055IMU imu;
    private DigitalChannel touch;

    //Переменные моторов
    private double zm1, zm2, zm3, zm4, zm5, zm6, zm7;
    private double zs1 = 0.71;

    //Переменные одометрии
    private Orientation angles;
    private double a, a_telescope, vyr, turn;
    double alpha;
    double m1v, m2v, m3v, m4v;
    double oldposy1 = 0, oldposy2 = 0, oldposx = 0,
            curposy1 = 0, curposy2 = 0, curposx = 0,
            ey1r, ey2r, exr,
            ex, ey,
            sina, cosa,
            beta, theta, sint, cost, sinb, cosb,
            posx = 0, posy = 0;

    //Работа энкодеров
    double prev_x = 0;
    double prev_y1 = 0;
    double prev_y2 = 0;
    double enc_x = 0;
    double enc_y1 = 0;
    double enc_y2 = 0;

    //Вольтаж
    private double voltage = 0, voltage_k;

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
        m1 = op.hardwareMap.get(DcMotor.class, "m1");
        m2 = op.hardwareMap.get(DcMotor.class, "m2");
        m3 = op.hardwareMap.get(DcMotor.class, "m3");
        m4 = op.hardwareMap.get(DcMotor.class, "m4");
        m5 = op.hardwareMap.get(DcMotor.class, "m5");

        s1 = op.hardwareMap.get(Servo.class, "s1");
        initIMU(op);

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
    }

    //Запись угла в файл
    public void writeAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, String.valueOf(angles.firstAngle));
    }

    public void measure(OpMode op) {
        this.op = op;

        double speed = 1;
        boolean slowmode = false;

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        while (opModeIsActive() && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            alpha = (angles.firstAngle + 450) % 360;

            oldposx = curposx;
            oldposy1 = curposy1;
            oldposy2 = curposy2;

            curposx = m1.getCurrentPosition() - prev_x;
            curposy1 = m2.getCurrentPosition() - prev_y1;
            curposy2 = m3.getCurrentPosition() - prev_y2;

            ey1r = curposy1 - oldposy1;
            ey2r = curposy2 - oldposy2;
            exr = curposx - oldposx;

            ey = (ey1r + ey2r) / 2;
            ex = exr + (ey2r - ey1r) / 2;

            sina = Math.sin(Math.toRadians(alpha));
            cosa = Math.cos(Math.toRadians(alpha));

            posx += ey*cosa + ex*sina;
            posy += ey*sina - ex*cosa;

            op.telemetry.addData("X:", posx);
            op.telemetry.addData("Y", posy);
            op.telemetry.addData("Alpha", alpha);
            op.telemetry.addLine("________________________________________________");
            op.telemetry.addData("Encoder X", curposx);
            op.telemetry.addData("Encoder Y1", curposy1);
            op.telemetry.addData("Encoder Y2", curposy2);
            op.telemetry.update();

        }
    }

    public void move(OpMode op, double x, double y, double spd, double ang, double timeout) {
        this.op = op;

        double speed = 1;
        double xl, yl, xv, yv, hypl, ugol = 0;
        boolean slowmode = false;

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        xl = x-posx;
        yl = y-posy;

        runtime.reset();

        while (opModeIsActive() && !isStopRequested() && (Math.abs(xl) > 1 || Math.abs(yl) > 1) && (runtime.milliseconds() - timeout * 1000) < 0) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            alpha = (angles.firstAngle + 450) % 360;

            oldposx = curposx;
            oldposy1 = curposy1;
            oldposy2 = curposy2;

            curposx = m1.getCurrentPosition() - prev_x;
            curposy1 = m2.getCurrentPosition() - prev_y1;
            curposy2 = m3.getCurrentPosition() - prev_y2;

            ey1r = curposy1 - oldposy1;
            ey2r = curposy2 - oldposy2;
            exr = curposx - oldposx;

            ey = (ey1r + ey2r) / 2;
            ex = exr + (ey2r - ey1r) / 2;

            sina = Math.sin(Math.toRadians(alpha));
            cosa = Math.cos(Math.toRadians(alpha));

            posx += ey*cosa + ex*sina;
            posy += ey*sina - ex*cosa;

            xl = x-posx;
            yl = y-posy;

            if (yl > 0) {
                if (xl >= 0) {
                    ugol = Math.atan(xl/yl);
                }
                if (xl < 0) {
                    ugol = 3.141592653589*2 - Math.abs(Math.atan(xl/yl));
                }
            }

            if (yl < 0) {
                if (x >= 0) {
                    ugol = 3.141592653589 - Math.abs(Math.atan(xl/yl));
                }
                if (xl < 0) {
                    ugol = 3.141592653589 + Math.atan(xl/yl);
                }
            }

            if (yl == 0) {
                if (xl >= 0) {
                    ugol = 3.141592653589/2;
                }
                if (xl < 0) {
                    ugol = -(3.141592653589/2);
                }
            }

            ugol = ugol - Math.toRadians(ang);

            vyr = (angles.firstAngle + ang) / 20;

            hypl = Math.sqrt(xl*xl + yl*yl);

            if (hypl <= 180) {
                slowmode = true;
            }
            if (slowmode == true) {
                spd = Range.clip(
                        ((0.22 + hypl/270)),
                        0.2,
                        0.5);
            }
            if (slowmode == false) {
                spd = 1;
            }

            voltage = op.hardwareMap.getAll(VoltageSensor.class).iterator().next().getVoltage();
            voltage_k = (12.5/voltage);

            theta = angles.firstAngle;
            beta = theta + 90;

            sinb = Math.sin(Math.toRadians(beta));
            cosb = Math.cos(Math.toRadians(beta));

            sint = Math.sin(Math.toRadians(theta));
            cost = Math.cos(Math.toRadians(theta));

            xv = Math.sin(ugol);
            yv = Math.cos(ugol);

            m1v = yv - xv;
            m2v = xv + yv;
            m3v = xv - yv;
            m4v = -xv - yv;

            //Headless mode
            /*
            m1.setPower(Range.clip(((sinb - cosb)*yv + (sint - cost)*xv - vyr), -1, 1) * spd * voltage_k);
            m2.setPower(Range.clip(((sinb + cosb)*yv + (sint + cost)*xv - vyr), -1, 1) * spd * voltage_k);
            m3.setPower(Range.clip(((-sinb + cosb)*yv + (-sint + cost)*xv - vyr), -1, 1) * spd * voltage_k);
            m4.setPower(Range.clip(((-sinb - cosb)*yv + (-sint - cost)*xv - vyr), -1, 1) * spd * voltage_k);
            */
            //Normal mode
            m1.setPower(Range.clip((m1v - vyr), -1, 1) * spd * voltage_k);
            m2.setPower(Range.clip((m2v - vyr), -1, 1) * spd * voltage_k);
            m3.setPower(Range.clip((m3v - vyr), -1, 1) * spd * voltage_k);
            m4.setPower(Range.clip((m4v - vyr), -1, 1) * spd * voltage_k);

            op.telemetry.addData("X:", posx);
            op.telemetry.addData("Y", posy);

            op.telemetry.addData("Alpha", alpha);
            op.telemetry.addLine("________________________________________________");

            op.telemetry.addData("Энкодер оси x (готовый)", m1.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y2", m2.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y1", m3.getCurrentPosition());

            op.telemetry.addData("Проех x", prev_x);
            op.telemetry.addData("Проех y1", prev_y1);
            op.telemetry.addData("Проех y2", prev_y2);

            op.telemetry.addData("Encoder X", curposx);
            op.telemetry.addData("Encoder Y1", curposy1);
            op.telemetry.addData("Encoder Y2", curposy2);
            op.telemetry.update();

        }
    }

    public void rotate(double ugolok) {

        int vyr_znak;

        ugolok = -ugolok;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        prev_x = m1.getCurrentPosition() + prev_x;
        prev_y1 = m2.getCurrentPosition() + prev_y1;
        prev_y2 = m3.getCurrentPosition() + prev_y2;

        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > 0.3) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            vyr = (angles.firstAngle - ugolok) / 150;
            vyr_znak = (int) (vyr/Math.abs(vyr));

            m1.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m2.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m3.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m4.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);

        prev_x = m1.getCurrentPosition() - prev_x;
        prev_y1 = m2.getCurrentPosition() - prev_y1;
        prev_y2 = m3.getCurrentPosition() - prev_y2;
    }

    public void rotate_rough(double ugolok) {

        int vyr_znak;

        ugolok = -ugolok;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > 1.5) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            vyr = (angles.firstAngle - ugolok) / 100;
            vyr_znak = (int) (vyr/Math.abs(vyr));

            m1.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m2.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m3.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
            m4.setPower(Range.clip(-(vyr + 0.2 * vyr_znak), -1, 1));
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }

    public void tele(double pos) {

        if (m5.getCurrentPosition() < pos) {
            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() < pos) {
                m5.setPower(-0.8);
            }
            m5.setPower(0);
        }

        if (m5.getCurrentPosition() > pos) {
            while (opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() > pos) {
                m5.setPower(0.3);
            }
            m5.setPower(0);
        }
    }

    public void rotate_zamer() {

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int vyr_znak;
        double ugolok = 90;

        ugolok = -ugolok;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        while (opModeIsActive() && !isStopRequested() && Math.abs(angles.firstAngle - ugolok) > -1) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            vyr = (angles.firstAngle - ugolok) / 150;
            vyr_znak = (int) (vyr/Math.abs(vyr));

            m1.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m2.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m3.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));
            m4.setPower(Range.clip(-(vyr + 0.15 * vyr_znak), -1, 1));

            op.telemetry.addData("Энкодер оси x", m1.getCurrentPosition() - prev_x);
            op.telemetry.addData("Энкодер оси y2", m2.getCurrentPosition() - prev_y1);
            op.telemetry.addData("Энкодер оси y1", m3.getCurrentPosition() - prev_y2);

            op.telemetry.addData("Энкодер оси x (истинный)", m1.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y1 (истинный)", m2.getCurrentPosition());
            op.telemetry.addData("Энкодер оси y2 (истинный)", m3.getCurrentPosition());
            op.telemetry.addData("Энкодер стрелы", m5.getCurrentPosition());
            op.telemetry.addData("Угол робота", angles.firstAngle);
            op.telemetry.update();
        }

        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }
}