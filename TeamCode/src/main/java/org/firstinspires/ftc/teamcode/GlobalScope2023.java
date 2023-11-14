package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public abstract class GlobalScope2023 extends LinearOpMode
{

    //------------------------------ANUNT-------------------------------
    // Toma nu mai ai dreptul de a mai modifica comentarii. -Cu drag, echipa roLERbot.
    //----------------------------FINAL ANUNT---------------------------
    public int brattpr= 1440;

    /*
    public double NEY = -0.7, NEX = 0.88, SEX = 0.83, SEY = 0.71;
    public double NWY = NEY, NWX = -NEX, SWX = -0.75, SWY = 0.81;
    public double AngleNE = 1.0 * NEX / NEY;
    public double AngleNW = 1.0 * NWX / NWY;
    public double AngleSE = 1.0 * SEX / SEY;
    public double AngleSW = 1.0 * SWX / SWY;
    */
    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx mb1 = null;
    public DcMotorEx mb2 = null;
    public Servo sj = null;
    public Servo rot = null;
    public Servo c1 = null;
    public Servo c2 = null;
    public BNO055IMU imu=null;
    double tickspercm = (537.7 / (Math.PI * 10));
    //double tickspercm = 1;
    int[] posbrat={
            -60, -60, -550, -800, -1440, -1440
        /// p0  p1   p2  p3 p4(invers) ultima e pentru o rutina ca sa scoti clestele din pozitia de start, vedeti in autonomie
    };
    int[] pbrat={
      0,-60 ,-550 ,-1350 , -1350
    };
    /*
    0.59, 0 - start
    0.45, 0  - ground
    0.46, 160 - lvl1
    0.49, 300 - lvl2
    0.52, 530 - lvl3
    0.47, 530 - lvl3 spate


    0.46, 20 - 2 conuri
    0.455, 40 - 3 conuri
    0.465, 55 - 4 conuri
    0.47, 83 - 5 conuri


    */
    double[] psj={
            0.45, 0.45, 0.46, 0.49, 0.52, 0.47
    };
    double[] psjauto={
            0.55-0.14, 0.55-0.13, 0.55-0.1, 0.55-0.075, 0.55-0.135, 0.55
    };
    void InitMotorsAuto()
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void StopMotorsRoti()
    {
        MotorFS.setPower(0);
        MotorFD.setPower(0);
        MotorSS.setPower(0);
        MotorSD.setPower(0);
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotorEx.class,"MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class,"MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class,"MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class,"MotorSD");
        mb1=hardwareMap.get(DcMotorEx.class,"mb1");
        mb2=hardwareMap.get(DcMotorEx.class,"mb2");
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setDirection(DcMotorSimple.Direction.REVERSE);
        //imu = hardwareMap.get(BNO055IMU.class,"imu");
        sj = hardwareMap.get(Servo.class, "sj");
        rot = hardwareMap.get(Servo.class, "rot");
        c1 = hardwareMap.get(Servo.class, "c1");
        c2 = hardwareMap.get(Servo.class, "c2");
        //------------------------------ANUNT-------------------------------
        // Toma nu mai ai dreptul de a mai modifica comentarii. -Cu drag, echipa roLERbot.
//        eu sunt rares si am voie
        //----------------------------FINAL ANUNT---------------------------
    }

    void schimbRotatie(SampleMecanumDrive drive) {
        drive.rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    void schimbInainte(SampleMecanumDrive drive) {
        drive.leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void schimbInainte() {
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    void Initialise()
    {
        LinkComponents();
        //------------------------------------
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);

        //--------------------------BRATZ-------------
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//
        mb1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//
        //mb1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//
        //mb2.setDirection(DcMotorSimple.Direction.REVERSE);
        mb1.setTargetPositionTolerance(10);
        mb2.setTargetPositionTolerance(10);
        MotorFS.setTargetPositionTolerance(10);
        MotorFD.setTargetPositionTolerance(10);
        MotorSS.setTargetPositionTolerance(10);
        MotorSD.setTargetPositionTolerance(10);
        mb2.setDirection(DcMotorSimple.Direction.FORWARD);
        mb1.setDirection(DcMotorSimple.Direction.REVERSE);
        //~~~~servouri
        c2.setDirection(Servo.Direction.REVERSE);

    }

    public void InitTeleop()
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Init_Motors_Auto()
    {
        MotorFS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    double maxf(double a, double b)
    {
        if (a > b)
            return a;
        return b;
    }

    double minf(double a, double b)
    {
        if (a < b)
            return a;
        return b;
    }

    double abs(double x)
    {
        if (x < 0)
            x = -x;
        return x;
    }
    /*
    void MoveByDpad()
    {
        telemetry.addData("Merge pe", "DPAD");
        telemetry.update();
        double speed;
        speed = (gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_up || gamepad1.dpad_right)?0.4:0.0;
        //speed = (gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right)?0.25:0.0;

        if(gamepad1.dpad_up || gamepad2.dpad_up) //sus
        {
            MotorFS.setPower(speed);
            MotorFD.setPower(speed);
            MotorSS.setPower(speed);
            MotorSD.setPower(speed);
        }
        else if (gamepad1.dpad_down || gamepad2.dpad_down) //jos
        {
            MotorFS.setPower(-speed);
            MotorFD.setPower(-speed);
            MotorSS.setPower(-speed);
            MotorSD.setPower(-speed);
        }
        else if (gamepad1.dpad_left || gamepad2.dpad_left) // stanga
        {
            MotorFS.setPower(-speed);
            MotorFD.setPower(speed);
            MotorSS.setPower(speed);
            MotorSD.setPower(-speed);
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right) //dreapta
        {
            MotorFS.setPower(speed);
            MotorFD.setPower(-speed);
            MotorSS.setPower(-speed);
            MotorSD.setPower(speed);
        }
        else StopMotors();
    }

    void MoveByBeteleFericirii() /// JoySticks
    {
        double speed;
        ///BatulDreapta
        double val = gamepad1.right_stick_y;
        if (val < 0.001 && val >= 0) val = 0.001;
        else if (val > -0.001 && val <= 0) val = -0.001;
        double angle = 1.0 * gamepad1.right_stick_x / gamepad1.right_stick_y;
        telemetry.addData("Tangenta unghiului este", angle);
        telemetry.update();
        if (gamepad1.right_stick_y < -deadzone) // AMOGUS - UPSIDE
        {
            //telemetry.addData("Am intrat", "in sus");
            //telemetry.update();
            if (gamepad1.right_stick_x > 0.0) // RightSide
            {
                //telemetry.addData("Am intrat", "in sus in dreapta");
                //telemetry.update();
                if (angle > AngleNE)
                {
                    //sus
                    //telemetry.addData("MERGE", "in sus");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_y);
                    MotorFS.setPower(speed);
                    MotorFD.setPower(speed);
                    MotorSS.setPower(speed);
                    MotorSD.setPower(speed);
                }
                else
                {
                    //telemetry.addData("MERGE", "in dreapta");
                    //telemetry.update();
                    //dreapta
                    speed = abs(gamepad1.right_stick_x);
                    MotorFS.setPower(speed);
                    MotorFD.setPower(-speed);
                    MotorSS.setPower(-speed);
                    MotorSD.setPower(speed);
                }
            }
            else //left
            {
                //telemetry.addData("Am intrat", "in sus in stanga");
                //telemetry.update();
                if (angle < AngleNW)
                {
                    //sus
                    //telemetry.addData("MERGE", "in sus");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_y);
                    MotorFS.setPower(speed);
                    MotorFD.setPower(speed);
                    MotorSS.setPower(speed);
                    MotorSD.setPower(speed);
                }
                else
                {
                    //stanga
                    //telemetry.addData("MERGE", "in stanga");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_x);
                    MotorFS.setPower(-speed);
                    MotorFD.setPower(speed);
                    MotorSS.setPower(speed);
                    MotorSD.setPower(-speed);
                }
            }
        }
        else if (gamepad1.right_stick_y > deadzone)
        {
            //telemetry.addData("Am intrat", "in jos");
            //telemetry.update();
            if (gamepad1.left_stick_x > 0.0)
            {
                telemetry.addData("Am intrat", "in jos in dreapta");
                telemetry.update();
                if (angle < AngleSE)
                {
                    //jos
                    //telemetry.addData("MERGE", "in jos");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_y);
                    MotorFS.setPower(-speed);
                    MotorFD.setPower(-speed);
                    MotorSS.setPower(-speed);
                    MotorSD.setPower(-speed);
                }
                else
                {
                    //dreapta
                    //telemetry.addData("MERGE", "in dreapta");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_x);
                    MotorFS.setPower(speed);
                    MotorFD.setPower(-speed);
                    MotorSS.setPower(-speed);
                    MotorSD.setPower(speed);
                }
            }
            else
            {
                //telemetry.addData("Am intrat", "in jos in stanga");
                //telemetry.update();
                if (angle > AngleSW)
                {
                    //jos
                    //telemetry.addData("MERGE", "in jos");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_y);
                    MotorFS.setPower(-speed);
                    MotorFD.setPower(-speed);
                    MotorSS.setPower(-speed);
                    MotorSD.setPower(-speed);
                }
                else
                {
                    //stanga
                    //telemetry.addData("MERGE", "in stanga");
                    //telemetry.update();
                    speed = abs(gamepad1.right_stick_x);
                    MotorFS.setPower(-speed);
                    MotorFD.setPower(speed);
                    MotorSS.setPower(speed);
                    MotorSD.setPower(-speed);
                }
            }
        }

        ///BatulStanga
        speed = maxf(abs(gamepad1.left_stick_x), abs(gamepad1.left_stick_y));
        if (gamepad1.left_stick_y > deadzone && gamepad1.left_stick_x > deadzone) //SE
        {
            MotorFD.setPower(-speed);
            MotorSS.setPower(-speed);
        }
        if (gamepad1.left_stick_y > deadzone && gamepad1.left_stick_x < -deadzone) //SW
        {
            MotorFS.setPower(-speed);
            MotorSD.setPower(-speed);
        }
        if (gamepad1.left_stick_y < -deadzone && gamepad1.left_stick_x > deadzone) //NE
        {
            MotorFS.setPower(speed);
            MotorSD.setPower(speed);
        }
        if (gamepad1.left_stick_y < -deadzone && gamepad1.left_stick_x < -deadzone) //NW
        {
            MotorFD.setPower(speed);
            MotorSS.setPower(speed);
        }
    }

    void MoveManuel()
    {

        if (gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down || gamepad1.dpad_up ||
                gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_down || gamepad2.dpad_up)
            MoveByDpad();

        if (abs(gamepad1.right_stick_x) > abs(deadzone) || abs(gamepad1.right_stick_y) > abs(deadzone) ||
                abs(gamepad1.left_stick_x) > abs(deadzone) || abs(gamepad1.left_stick_y) > abs(deadzone) ||
                abs(gamepad2.right_stick_x) > abs(deadzone) || abs(gamepad2.right_stick_y) > abs(deadzone) ||
                abs(gamepad2.left_stick_x) > abs(deadzone) || abs(gamepad2.left_stick_y) > abs(deadzone))
            MoveByBeteleFericirii();

    }
    */


    void StopMotors()
    {
        MotorFS.setPower(0.0);
        MotorFD.setPower(0.0);
        MotorSS.setPower(0.0);
        MotorSD.setPower(0.0);
    }


    /// FACE PUIU DRIFFFTTTTTTTURI KA LA KAUFLAND
    /*
    void LiftBS()
    {
        if (abs(gamepad2.left_stick_y)>0.1)
            MotorBS.setPower(-gamepad2.left_stick_y*0.6);
        else StopBS();
    }
    void LiftBD()
    {
        if (abs(gamepad2.right_stick_y)>0.1) {
            MotorBD.setPower(0.8*gamepad2.right_stick_y);
            telemetry.addData("s-a ridicat ", 2);
        }
        else StopBD();
    }

    Thread servo = new Thread(() -> {
        double startpos = ServoBrat.getPosition();
        while(!Thread.currentThread().isInterrupted()) {
            while (gamepad1.x) {
                ServoBrat.setDirection(Servo.Direction.REVERSE);
                ServoBrat.setPosition(startpos + 0.2);
            }
            if(ServoBrat.getPosition()!=startpos && !gamepad1.x) {
                ServoBrat.setDirection(Servo.Direction.REVERSE);
                ServoBrat.setPosition(startpos);
            }
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    });
    */
    void MoveAuto(float distance, char c, double speed)
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);
        idk(distance,c);
        MotorFS.setPower(speed);
        MotorFD.setPower(speed);
        MotorSS.setPower(speed);
        MotorSD.setPower(speed);
        while(MotorSS.isBusy() && MotorFD.isBusy()
                && MotorFS.isBusy() && MotorSD.isBusy())
            ;
        StopMotorsRoti();
    }
    void idk(double distance, char c)
    {
        if(c =='F') {
            MotorFS.setTargetPosition((int) (distance * tickspercm));
            MotorFD.setTargetPosition((int) (distance * tickspercm));
            MotorSS.setTargetPosition((int) (distance * tickspercm));
            MotorSD.setTargetPosition((int) (distance * tickspercm));
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (c == 'D')
        {
            MotorFS.setTargetPosition((int) (-distance * tickspercm));
            MotorFD.setTargetPosition((int) (-distance * tickspercm));
            MotorSS.setTargetPosition((int) (-distance * tickspercm));
            MotorSD.setTargetPosition((int) (-distance * tickspercm));
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (c == 'R')
        {
            MotorFS.setTargetPosition((int) (distance * tickspercm));
            MotorFD.setTargetPosition((int) (-distance * tickspercm));
            MotorSS.setTargetPosition((int) (-distance * tickspercm));
            MotorSD.setTargetPosition((int) (distance * tickspercm));
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if (c == 'L')
        {
            MotorFS.setTargetPosition((int) (-distance * tickspercm));
            MotorFD.setTargetPosition((int) (distance * tickspercm));
            MotorSS.setTargetPosition((int) (distance * tickspercm));
            MotorSD.setTargetPosition((int) (-distance * tickspercm));
            MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        /**
        MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         */
    }
}
