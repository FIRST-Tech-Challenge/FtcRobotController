package org.firstinspires.ftc.teamcode;
//package org.firstinspires.ftc.robotcontroller.external.samples;

import android.os.Build;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;


///practic aici doar declaram obiectele care se folosesc si in TeleOp dar si in auto
public abstract class GlobalScope extends LinearOpMode { 
    public class Controller {
        //public double x, y, a, b,
        //dpad_up,dpad_down,dpad_left,dpad_right,
        //lstick_x,
        //rstick_x, rstick_y,
        //rbumper, lbumper,
        //rtrigger, ltrigger;
        public double lstick_y;
        public double bumpers;
    }
    public int[] pozitie={200,400,650};
    public float rata;
    public float unghiZero,unghiFix, currpos;
    //int fronturi = 10;
    int bubuiala = 36; // !!! ASTA E CONSTANTA !!! (din nou ig)
    int position1 = 0;
    int position2 = 0;
    int position3 = 0;
    int position4 = 0;
    int pozitiecur1 = 0; // aka pozitia curenta, pozitia curului e la ionut, mereu 7 fronturi
    int pozitiecur2 = 0;
    int pozitiecur3 = 0;
    int pozitiecur4 = 0;
    public int posBrat;
    //public ColorSensor senzorCuloare=null;
    public DcMotor MotorFS=null;
    public DcMotor MotorFD=null;
    public DcMotor MotorSS=null;
    public DcMotor MotorSD=null;
    public DcMotor MotorRoataRata =null;
    public DcMotor MotorBrat=null;
    public DcMotor MotorLateral=null;
    public ColorSensor SenzorP1=null;
    public ColorSensor SenzorP2=null;
    public int LiftPos=0;
    public CRServo ServoBrat=null;
    BNO055IMU Gyro = null;
    void InitMotorsAuto()
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void Lift(boolean up,boolean down,boolean x) { //like, do you even?
        telemetry.addData("ba ","nu");
        if(x)ServoBrat.setPower(2.0);
        else ServoBrat.setPower(-1.0);
        ///double powerbrat=(gamepad1.left_trigger>0.4)?gamepad1.left_trigger:0.4;
        if(down)MotorBrat.setPower(0.7);
        else if(up)MotorBrat.setPower(-0.7);
        //if(MotorBrat.getCurrentPosition()==posBrat)
        ///    MotorBrat.setPowe\r(0.05);
        ///  MotorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //while(MotorBrat.isBusy()) ;
    }

    void Miltii()
    {
        MotorBrat.setPower(0.0);
        ServoBrat.setPower(-1.0);
    }

    void StopMotorsRoti() {
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
    }

    void RotateRatoi(double speed)
    {
        MotorRoataRata.setPower(speed);
    }

    void RotateRata(double speed)
    {
        MotorRoataRata.setPower(speed);
        sleep(4000);
        MotorRoataRata.setPower(0);
    }
    void Rotate(double speed, double time) {
        MotorFS.setPower(speed);
        MotorFD.setPower(speed);
        MotorSS.setPower(speed);
        MotorSD.setPower(speed);
        sleep((long) (time * 1000));
        if (time != 0)
            StopMotorsRoti();
    }

    void MoveFS(double speed, double time) { // FS vine de la Florin Salam
        MotorFS.setPower(speed);
        MotorFD.setPower(-speed);
        MotorSS.setPower(speed);
        MotorSD.setPower(-speed);
        sleep((long) (time * 1000));
        if (time != 0)
            StopMotorsRoti();
    }
    void MoveManuel()
    {
        double speed = gamepad1.right_trigger + gamepad2.left_stick_y * 0.2;/// + gamepad1.left_trigger * 0.3;
        if(gamepad1.dpad_down || gamepad2.left_stick_y<=-0.2)
        {
            MotorFS.setPower(speed);
            MotorFD.setPower(-speed);
            MotorSS.setPower(speed);
            MotorSD.setPower(-speed);
        }
        else if (gamepad1.dpad_up || gamepad2.left_stick_y>=0.2)
        {
            MotorFS.setPower(-speed);
            MotorFD.setPower(speed);
            MotorSS.setPower(-speed);
            MotorSD.setPower(speed);
        }
        else TrrFa();
    }

    void TrrFa()
    {
        MotorFS.setPower(0.0);
        MotorFD.setPower(0.0);
        MotorSS.setPower(0.0);
        MotorSD.setPower(0.0);
        MotorLateral.setPower(0.0);
    }

    void Initialise() {
        LinkComponents();
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorLateral.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorBrat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //MotorBrat.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorRoataRata.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //MotorBrat.setPower(1);
    }
    void LinkComponents() {
        ///Culoare=hardwareMap.get(ColorSensor.class, "senzorCuloare");
        MotorFS= hardwareMap.get(DcMotor.class,"MotorFS");
        MotorFD= hardwareMap.get(DcMotor.class,"MotorFD");
        MotorSS= hardwareMap.get(DcMotor.class,"MotorSS");
        MotorSD= hardwareMap.get(DcMotor.class,"MotorSD");
        MotorLateral= hardwareMap.get(DcMotor.class,"MotorLateral");
        MotorRoataRata= hardwareMap.get(DcMotor.class,"MotorRoataRata");
        MotorBrat= hardwareMap.get(DcMotor.class,"MotorBrat");
        ServoBrat= hardwareMap.get(CRServo.class,"ServoBrat");
        SenzorP1= hardwareMap.get(ColorSensor.class,"SenzorP1");
        SenzorP2= hardwareMap.get(ColorSensor.class,"SenzorP2");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "Calibrare1.json";       /**Trebuie schimbat cu denumirea filei*/
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        Gyro = hardwareMap.get(BNO055IMU.class, "imu");
        Gyro.initialize(parameters);
        MotorRoataRata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //de aici e exit point-ul in functie, somehow
        System.gc();// rugam frumos miraculosul java garbage collector sa stearga variabilele,
        // pentru ca nu le mai folosim
    }

    int Rotunjire(double x) {
        if(x>0.3)
            return 1;
        else if (x<-0.3)
            return -1;
        return 0;
    }

    public void Init_Motors_Auto() {

        position1 = position2 = position3 = position4 = 0;
        MotorFS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorBrat.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void Move(int distance, double power, char direction) {
        int position;
        // distance = (int) (distance * 1.05);
        position = distance * bubuiala;
        position1 = MotorFS.getCurrentPosition();
        position2 = MotorFD.getCurrentPosition();
        position3 = MotorSS.getCurrentPosition();
        position4 = MotorSD.getCurrentPosition();
        if (direction == 'F')
        {
            position1 = position1 - position;
            position2 = position2 + position;
            position3 = position3 - position;
            position4 = position4 + position;
        }
        else if(direction == 'B')
        {
            position1 = position1 + position;
            position2 = position2 - position;
            position3 = position3 + position;
            position4 = position4 - position;
        }
        MotorFS.setTargetPosition(position1);
        MotorFD.setTargetPosition(position2);
        MotorSS.setTargetPosition(position3);
        MotorSD.setTargetPosition(position4);

        MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFS.setPower(power);
        MotorFD.setPower(power);
        MotorSS.setPower(power);
        MotorSD.setPower(power);
        pozitiecur1 = MotorFS.getCurrentPosition();
        pozitiecur2 = MotorFD.getCurrentPosition();
        pozitiecur3 = MotorSS.getCurrentPosition();
        pozitiecur4 = MotorSD.getCurrentPosition();
        while(MotorFS.isBusy() && MotorFD.isBusy()
                && MotorSS.isBusy() && MotorSD.isBusy());
    }

    void RotateAuto(double position,float speed)
    {
        double angle;
        unghiZero = Gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle
                + 180;
        angle = unghiZero - position;
        if (angle < 0) angle = -angle;
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setPower(speed);
        MotorFD.setPower(speed);
        MotorSS.setPower(speed);
        MotorSD.setPower(speed);
        while (Gyro.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180 > angle + 3 ||
                Gyro.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180 < angle - 3 ) {

        }
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
    }

    void RotateAutoFix(double position,float speed)
    {
        double angle;
        angle = unghiFix - position;
        if (angle < 0) angle = -angle;
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFS.setPower(speed);
        MotorFD.setPower(speed);
        MotorSS.setPower(speed);
        MotorSD.setPower(speed);
        while (Gyro.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180 > angle + 3 ||
                Gyro.getAngularOrientation(AxesReference.INTRINSIC,
                        AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle + 180 < angle - 3 ) {

        }
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
    }

    void Catch()
    {
        MotorBrat.setPower(0.3);
        MotorBrat.setTargetPosition(180);
        ServoBrat.setPower(-1.0);
        ServoBrat.setDirection(DcMotorSimple.Direction.FORWARD);
        sleep(1000);

    }

    void Ridicare(int nivel, float susjos)
    {

        // am pus puterea motorului la 0.3 ca sa nu innebuneasca,
        //daca nu are destula putere sa se ridice il ajutati putin cu mana/ modificati voi valoarea
        // nici pe plus nici pe minus... refuza sa fie util
        //posBrat = MotorBrat.getCurrentPosition();
        // am afisat pozitia curenta inainte si in timpul miscarii.
        // aveti grija ca angle ul sa nu fie in directia gresita.
        MotorBrat.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ////MotorBrat.setTargetPosition(angle + posBrat);
        MotorBrat.setPower(-0.70*susjos);
        /**
         pos     pow   sleep
         sus -> -0.70, 650
         mij -> -0.70, 380
         jos -> -0.70, 160

         ESTIMATIV!! (puteti modifica in functie de cat de deschis e servo-ul)
         * */
        sleep(pozitie[nivel]);
        MotorBrat.setPower(0);
        sleep(50);
    }

    void Release()
    {
        ServoBrat.setPower(1.0);
    }

}
