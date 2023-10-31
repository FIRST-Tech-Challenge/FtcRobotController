package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.util.ArrayList;


@Disabled
@Autonomous
public class AutoMethods extends LinearOpMode implements Inter{

    public OpenCvCamera webcam;
    public boolean camError = false;
    public ElapsedTime runtime = new ElapsedTime();
    public OpMode op;
    public int baza = 1;
    private BNO055IMU imu;
    private Orientation angles;

    File getAngleFile;
    File angleFile = AppUtil.getInstance().getSettingsFile("angle.txt"); //Файл с позицией робота
    //Железо
    private double hyi = 0;
    private DcMotor m1, m2, m3, m4, m5, EnX1,EnY2,EnY3;
    public Servo s5;
    private DigitalChannel touch;
    private int height;
    private double zm5;
    //Переменные мотора
    private int sleep = 350;
    public  enum  Direction{
        forward,
        back,
        left,
        right
    }
    @Override
    public void runOpMode() throws InterruptedException {
    }
    //Инициализируем гироскоп
    public void initIMU(OpMode op) {
        this.op = op;
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = op.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //Запись угла в файл
    public void writeAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ReadWriteFile.writeFile(angleFile, String.valueOf(angles.firstAngle));
    }
    //Инициализируем железо
    public void initC(OpMode op) {
        this.op = op;
        m1 = op.hardwareMap.get(DcMotor.class, "m1");
        m2 = op.hardwareMap.get(DcMotor.class, "m2");
        m3 = op.hardwareMap.get(DcMotor.class, "m3");
        m4 = op.hardwareMap.get(DcMotor.class, "m4");
        m5 = op.hardwareMap.get(DcMotor.class, "m5");

        s5 = op.hardwareMap.get(Servo.class, "s5");

        s5.setPosition(OPEN);

        EnX1 = op.hardwareMap.get(DcMotor.class, "En1");
        EnY2 = op.hardwareMap.get(DcMotor.class, "En2");
        EnY3 = op.hardwareMap.get(DcMotor.class, "En3");

        m1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m5.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        EnX1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EnY2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        EnY3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        m1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        EnX1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EnY2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EnY3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m5.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EnX1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EnY2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        EnY3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        EnY2.setDirection(DcMotorSimple.Direction.REVERSE);

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
            webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        }catch (OpenCvCameraException e){camError = true;}
        catch (NullPointerException e2){camError = true;}
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
    public void drive (int X, int Y, OpMode op, double timeout) {
        this.op = op;

        EnY2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EnY3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int X_rasst = X;
        int Y_rasst = Y;
        double spdX = 0, spdY = 0;
        double encX = EnX1.getCurrentPosition();
        double encY = -(EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 2.;
        double ostX = X_rasst-encX;
        double ostY =Y_rasst-encY;
        double SquareGip = (ostX*ostX) + (ostY*ostY);
        double Gip = Math.sqrt(SquareGip);
        double turn = 0;

        //PID
        double P = 0;
        double I = 0;
        double D = 0;

        double Kp = 0.00025;
        double Ki = 0;
        double Kd = 0.0005;

        double last_time = 0;
        double last_gip = Gip;

        double PID = 0;

        runtime.reset();


        while (!isStopRequested() && !opModeIsActive() && runtime.seconds() < timeout && Gip > hyi) {

            P = Kp * Gip;

            I += Ki * Gip;

            if (runtime.milliseconds() - last_time > 50)
            {
                D = Kd * (Gip - last_gip);
                last_time = runtime.milliseconds();
                last_gip = Gip;
            }

            PID = P + I + D;

            encX = EnX1.getCurrentPosition();
            encY = (EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 2.;

            SquareGip = (ostX*ostX) + (ostY*ostY);
            Gip = Math.sqrt(SquareGip);

            ostX = X_rasst-encX;
            ostY = Y_rasst-encY;

            spdX = ostX / Gip;
            spdY = ostY / Gip;

            turn = (-EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 800.0;

            m1.setPower((-spdX + spdY - turn) * PID);
            m2.setPower((-spdX - spdY - turn) * PID);
            m3.setPower((spdX - spdY - turn) * PID);
            m4.setPower((spdX + spdY - turn) * PID);

            op.telemetry.addData("spdX", spdX);
            op.telemetry.addData("spdY", spdY);
            op.telemetry.addData("Speed", PID);

            op.telemetry.addData("X", EnX1.getCurrentPosition());
            op.telemetry.addData("Y", (EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 2);

            op.telemetry.addData("EnY2", EnY2.getCurrentPosition() );
            op.telemetry.addData("EnY3", EnY3.getCurrentPosition() );


            op.telemetry.addData("encX", encX);
            op.telemetry.addData("encY", encY);
            op.telemetry.addData("ostX", ostX);
            op.telemetry.addData("ostY", ostY);
            op.telemetry.addData("En1", EnX1.getCurrentPosition());
            op.telemetry.addData("En2", EnY2.getCurrentPosition());
            op.telemetry.addData("En3", EnY3.getCurrentPosition());
            op.telemetry.update();

        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
       // sleep(10000);

    }





    public void turn(int ang, OpMode op) {
        this.op = op;
        double angost;
        double speed = 0.3;
//        if(ang>180){
//
//            angost=Math.abs(ang-360);
//
//        }
//        else {
            angost=ang;

        EnY2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        EnY3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (!isStopRequested() && !opModeIsActive() && angost > Math.abs(((EnY2.getCurrentPosition() - EnY3.getCurrentPosition()) / 16.5 )/2.)){
           if(ang < 180) {
                 m1.setPower(speed);
                 m2.setPower(speed);
                 m3.setPower(speed);
                 m4.setPower(speed);
                   op.telemetry.addData("ang", Math.abs(((EnY2.getCurrentPosition() - EnY3.getCurrentPosition()) / 16.5) / 2.));
                   op.telemetry.update();

            }

           if(ang == 180 ) {
               m1.setPower(-speed);
               m2.setPower(-speed);
               m3.setPower(-speed);
               m4.setPower(-speed);
               op.telemetry.addData("ang", Math.abs(((EnY2.getCurrentPosition() - EnY3.getCurrentPosition()) / 16.5) / 2.));
               op.telemetry.update();}
            if(ang>180) {
                m1.setPower(-speed);
                m2.setPower(-speed);
                m3.setPower(-speed);
                m4.setPower(-speed);
                op.telemetry.addData("ang", Math.abs(((EnY2.getCurrentPosition() - EnY3.getCurrentPosition()) / 16.5) / 2.));
                op.telemetry.update();
            }
        }
        m1.setPower(0);
        m2.setPower(0);
        m3.setPower(0);
        m4.setPower(0);
    }






    public void drive_passive (int X, int Y, double speed, OpMode op) {
        this.op = op;
        double X_rasst = X;
        double Y_rasst = Y;
        double spdX = 0, spdY = 0;
        double encX = EnX1.getCurrentPosition();
        double encY = -(EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 2.;
        double ostX = X_rasst-encX;
        double ostY = Y_rasst-encY;
        double SquareGip = (ostX*ostX) + (ostY*ostY);
        double Gip = Math.sqrt(SquareGip);
        boolean tormoz = false;
        boolean up = false;
        runtime.reset();


        if (ostX > ostY) {
            spdY = ostY/Math.abs(ostX);
            spdX = 1;
        }
        if (ostY > ostX) {
            spdX = ostX/Math.abs(ostY);
            spdY = 1;
        }
        if (ostX  == ostY) {
            spdX = 1;
            spdY = 1;
        }
        if(Gip < 1000){
            tormoz = true;
        }

        while (!isStopRequested() && opModeIsActive()) {

            if (tormoz) {
                speed = speed / 1.5;
            }

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            encX = EnX1.getCurrentPosition();
            encY = -(EnY2.getCurrentPosition()  + EnY3.getCurrentPosition()) / 2;

            SquareGip = (ostX*ostX) + (ostY*ostY);
            Gip = Math.sqrt(SquareGip);

            ostX = X_rasst-encX;
            ostY = Y_rasst-encY;
            if (ostX > ostY) {
                spdY = ostY/Math.abs(ostX);
                spdX = 1;
            }
            if (ostY > ostX) {
                spdX = ostX/Math.abs(ostY);
                spdY = 1;
            }
            if (ostX  == ostY) {
                spdX = 1;
                spdY = 1;
            }

            if (Gip < 1000) {
                tormoz = true;
            }

            op.telemetry.addData("spdX", spdX);
            op.telemetry.addData("spdY", spdY);

            op.telemetry.addData("X", EnX1.getCurrentPosition());
            op.telemetry.addData("Y", -(EnY2.getCurrentPosition() + EnY3.getCurrentPosition()) / 2);

            op.telemetry.addData("encX", encX);
            op.telemetry.addData("encY", encY);
            op.telemetry.addData("ostX", ostX);
            op.telemetry.addData("ostY", ostY);
            op.telemetry.addData("En1", EnX1.getCurrentPosition());
            op.telemetry.addData("En2", EnY2.getCurrentPosition());
            op.telemetry.addData("En3", EnY3.getCurrentPosition());
            op.telemetry.addData("Baza", baza);
            op.telemetry.addData("Gip", Gip);
            op.telemetry.addData("m5", m5.getCurrentPosition());
            op.telemetry.addData("Уровень", height);
            op.telemetry.addData("Гироскоп", angles.firstAngle);
            op.telemetry.addData("Up", up);
            op.telemetry.addData("ANG", (((EnY2.getCurrentPosition() - EnY3.getCurrentPosition()) / 18 ))/2);
            op.telemetry.update();
        }
    }
//
//    public void angleft(int ang) {
//        while ((EnY3.getCurrentPosition() + EnY2.getCurrentPosition()) / 2 < ang) {
//
//
//        }
//
//
//    }

    public void close(){
        s5.setPosition(CLOSE);
    }



    public void open(){

        s5.setPosition(OPEN);
    }


    public void Telescope (int number){
        while(!opModeIsActive() && !isStopRequested() && m5.getCurrentPosition() != number){
            if (number - m5.getCurrentPosition() > 10) {
                m5.setPower(-0.75);
            }
            else if (number - m5.getCurrentPosition() < -10) {
                m5.setPower(0.5);
            }
            else {
                m5.setPower(-0.05);
                break;
            }
        }
    }
}
