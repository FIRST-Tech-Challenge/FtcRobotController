/// REGULA NR 1 - TOMA NU SCRIE COD PERFECT
/// REGULA NR 2 - afirmati regula nr 1
/// REGULA NR 3 - puiu e cel mai smecher

package org.firstinspires.ftc.teamcode2024;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.acmerobotics.roadrunner.geometry.Pose2d;

/// Steal the pose

public abstract class GlobalScope2024 extends LinearOpMode
{
    /// GENERAL SYSTEMS
    public double currentHeading;
    public DcMotorEx MotorFS = null; /// Fata stanga
    public DcMotorEx MotorFD = null; /// Fata dreapta
    public DcMotorEx MotorSS = null; /// Spate stanga
    public DcMotorEx MotorSD = null; /// Spate dreapta
    public DcMotorEx mb1 = null; /// motor brat 1 control hub port 1, motor stanga
    public DcMotorEx mb2 = null; /// motor brat 2 control hub port 2, motor dreapta
    public DcMotorEx ky5_2 = null;
    public DcMotorEx ky5_1 = null;
    public Servo SDrone = null; // lanseaza avion
    public Servo SCutie = null;
    public Servo SPixel = null;
    public Servo SCamera = null;

    void LinkComponents()
    {
        MotorFS = hardwareMap.get(DcMotorEx.class,"MotorFS");
        MotorFD = hardwareMap.get(DcMotorEx.class,"MotorFD");
        MotorSS = hardwareMap.get(DcMotorEx.class,"MotorSS");
        MotorSD = hardwareMap.get(DcMotorEx.class,"MotorSD");
        mb1 = hardwareMap.get(DcMotorEx.class,"mb1");
        mb2 = hardwareMap.get(DcMotorEx.class,"mb2");
        ky5_1 = hardwareMap.get(DcMotorEx.class,"ky5_1");
        ky5_2 = hardwareMap.get(DcMotorEx.class,"ky5_2");
        SDrone = hardwareMap.get(Servo.class, "SDrone");
        SCutie = hardwareMap.get(Servo.class, "SCutie");
        SPixel = hardwareMap.get(Servo.class, "SPixel");
        SCamera = hardwareMap.get(Servo.class, "SCamera");
    }

    void Initialise()
    {
        LinkComponents();
        //---------------------ROTZI---------------
        MotorFS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSS.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorSD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorSD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorFS.setTargetPositionTolerance(5); /// Eroare de 10 unitati I guess idk
        MotorFD.setTargetPositionTolerance(5);
        MotorSS.setTargetPositionTolerance(5);
        MotorSD.setTargetPositionTolerance(5);
        MotorFS.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorSS.setDirection(DcMotorSimple.Direction.REVERSE);

        //--------------------------BRATZ-------------
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); ///mereu sa fie pe brake
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ky5_1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ky5_2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//
        mb2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//
        mb1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ky5_1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ky5_2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ky5_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ky5_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ky5_1.setDirection(DcMotorSimple.Direction.FORWARD);
        ky5_2.setDirection(DcMotorSimple.Direction.REVERSE);
        SCamera.setDirection(Servo.Direction.FORWARD);

        mb2.setDirection(DcMotorSimple.Direction.REVERSE); //vedem daca trebuie sa schibam directia la teste
        mb1.setDirection(DcMotorSimple.Direction.FORWARD);
        SCutie.setDirection(Servo.Direction.FORWARD);

        //--------------AVION-------
        SDrone.setDirection(Servo.Direction.FORWARD); /// hopefully - daca face invers schimbam pe reverse
        //-------------PIXEL------
        SPixel.setDirection(Servo.Direction.REVERSE);
    }

    void StopMotors()
    {


        MotorFS.setPower(0.0);
        MotorFD.setPower(0.0);
        MotorSS.setPower(0.0);
        MotorSD.setPower(0.0);
    }

    /// TELEOP

    double drive;
    double strafe;
    double twist;
    double[] speeds = new double[4];
    double schimbator = 0.4;
    double schimbator_brat = 0.8;
    boolean YEET_THE_DRONE = false; /// Devine 1 daca a fost lansata drona
    boolean Leave = false;
    boolean stateRotire;
    boolean clesteDreapta = true, clesteStanga = true;
    double c1Poz, c2Poz;
    int poz;
    int IsHanged = 0, IsDetached = 0, IlTin = 0;
    double SCutieArray[] = {0, 0.36, 0.432, 0.37};
    int  BratArray[] = {0, 0, 495, 495, 560, 700};///495
    GamepadEx ct1;
    GamepadEx ct2;
    ButtonReader LeftClesteOpener, RightClesteOpener;
    ButtonReader IAMSPEED; /// cautator de viteze
    ButtonReader Launch;
    ButtonReader Pixel;
    ButtonReader ARMISSPEED;
    ButtonReader ArmUp, ArmDown;
    ButtonReader Hang;
    ButtonReader Fall;
    ButtonReader Detach;
    ButtonReader TurnToggle;
    ButtonReader LTurn, RTurn;
    /// THIS IS NOT A JOJO REFERENCE
    ButtonReader Aerosmith;
    ButtonReader Oasis;
    // THERE ARE TWO JOJO REFERENCES

    void WeGottaMove()
    {
        IAMSPEED.readValue();
        TurnToggle.readValue();
       // telemetry.addData("viteza este", schimbator);
        //telemetry.update();
        if(IAMSPEED.wasJustPressed())
        {
            schimbator = 1.10 - schimbator; ///deocamdata avem doar 2 viteze dar daca e nevoie de mai multe facem
            telemetry.addData("viteza este", schimbator);
            telemetry.update();
        }
        drive  = -gamepad1.left_stick_y * schimbator;//-gamepad1.left_stick_y*0.3-
        strafe = gamepad1.left_stick_x * schimbator;//gamepad1.left_stick_x*0.3b
        //if(TurnToggle.wasJustPressed())
            twist  = schimbator*(gamepad1.right_trigger-gamepad1.left_trigger) ;
        //else
        //{
      //      if(RTurn.wasJustPressed())
            //    drive.turn(Math.toRadians(currentHeading+ 90));
      //  }
        speeds[0]=(drive + strafe + twist);//FS
        speeds[1]=(drive - strafe - twist);//FD
        speeds[2]=(drive - strafe + twist);//SS
        speeds[3]=(drive + strafe - twist);//SD
        ///telemetry.addData("axa y", gamepad1.right_stick_y);
        ///telemetry.addData("axa x", gamepad1.right_stick_x);
        ///telemetry.update();
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }
        // apply the calculated values to the motors.
        MotorFS.setPower(speeds[0]);
        MotorFD.setPower(speeds[1]);
        MotorSS.setPower(speeds[2]);
        MotorSD.setPower(speeds[3]);
    }

    void InitMiscare() /// chestiile de trebuie sa le pui de fiecare data ca sa se miste
    {
        MotorFS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSS.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorSD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFS.setPower(0.3);
        MotorFD.setPower(0.3);
        MotorSS.setPower(0.3);
        MotorSD.setPower(0.3);
        while ((MotorFS.isBusy() && MotorFD.isBusy() && MotorSD.isBusy() && MotorSS.isBusy()) && !isStopRequested())
        {
            telemetry.addData("Pozitita", MotorFS.getCurrentPosition());
            telemetry.update();
        }
    }

    void RotateStanga() /// Numele se explica singur
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - 1100);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + 1100);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - 1100);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + 1100);
        InitMiscare();
    }

    void RotateDreapta() /// Numele se explica singur
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + 1050);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - 1050);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + 1050);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - 1050);
        InitMiscare();
    }

    void MiscareStanga(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - x);
        InitMiscare();
    }
    void MiscareDreapta(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + x);
        InitMiscare();
    }
    void MiscareFata(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() + x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() + x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() + x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() + x);
        InitMiscare();
    }

    void MiscareSpate(int x)
    {
        MotorFS.setTargetPosition(MotorFS.getCurrentPosition() - x);
        MotorFD.setTargetPosition(MotorFD.getCurrentPosition() - x);
        MotorSS.setTargetPosition(MotorSS.getCurrentPosition() - x);
        MotorSD.setTargetPosition(MotorSD.getCurrentPosition() - x);
        InitMiscare();
    }

    void SetPozBrat(int poz)
    {
        mb1.setTargetPosition(BratArray[poz]);
        mb2.setTargetPosition(BratArray[poz]);
        mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mb1.setPower(0.1);
        mb2.setPower(0.1);
        sleep(500);
        SCutie.setPosition(SCutieArray[poz]);
        while (mb1.isBusy() && !isStopRequested()) {
            telemetry.addData("pozitia brat", mb1.getCurrentPosition());
            telemetry.update();
        }
    }

    void WeGottaLift()
    {
        ARMISSPEED.readValue();
        //telemetry.addData("viteza bratului este", schimbator_brat);
        //telemetry.update();
        if(ARMISSPEED.wasJustPressed())
        {
            schimbator_brat = 1.4 - schimbator_brat; ///deocamdata avem doar 2 viteze dar daca e nevoie de mai multe facem
            telemetry.addData("viteza bratului este", schimbator_brat);
            telemetry.update();
        }
        if (gamepad2.dpad_left)
        {
            telemetry.addData("bRAT SUS", mb1.getCurrentPosition());
            telemetry.addData("bRAT SUS", mb2.getCurrentPosition());
            telemetry.update();
            mb1.setPower(0.5);
            mb2.setPower(0.5);
        }
        else
        if (gamepad2.dpad_right)
        {
            telemetry.addData("bRAT SUS", mb1.getCurrentPosition());
            telemetry.addData("bRAT SUS", mb2.getCurrentPosition());
            telemetry.update();
            mb1.setPower(-0.25);
            mb2.setPower(-0.25);
        }
        else
        {
            mb1.setPower(0);
            mb2.setPower(0);
        }
    }

    void WeGottaLunchDrone()
    {
        Launch.readValue();
        if (Launch.wasJustPressed())
        {
            telemetry.addData("RUMBLLLLLLLLLLLIIIIINGGG",  "ITS COOOOOOOMMMMMMMIIIIIIIINNNNNNGG");
            telemetry.update();
            if (YEET_THE_DRONE == false)
            {
                SDrone.setPosition(0.4); /// vedem daca 0.3 e bine, deocamdata e pus la misto
                sleep(1500);
                YEET_THE_DRONE = true;
                SDrone.setPosition(0.6);
            }
        }
    }


    void ToggleArmUpDown(){
        ArmUp.readValue();
        if (ArmUp.wasJustPressed()) {
            SCutie.setPosition(0.35);
        }
        ArmDown.readValue();
        if (ArmDown.wasJustPressed()) {
            SCutie.setPosition(0.432);
        }
    }


    void WeGottaDoArmMovements()
    {
        Aerosmith.readValue();
        if (Aerosmith.wasJustPressed() && poz < 3)
        {
            poz++;
            if (poz == 3)
            {
                SCutie.setPosition(SCutieArray[poz]);
            }
            else
            {
                mb1.setTargetPosition(BratArray[poz]);
                mb2.setTargetPosition(BratArray[poz]);
                mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb1.setPower(0.2);
                mb2.setPower(0.2);
                sleep(500);
                SCutie.setPosition(SCutieArray[poz]);
                while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                    telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                    telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                    telemetry.update();
                }
            }
            telemetry.addData("pozitia brat", mb1.getCurrentPosition());
            telemetry.update();
        }
        Oasis.readValue();
        if (Oasis.wasJustPressed() && poz > 1)
        {
            poz--;
            if (poz == 2)
            {
                SCutie.setPosition(SCutieArray[poz]);
            }
            else
            {
                mb1.setTargetPosition(BratArray[poz]);
                mb2.setTargetPosition(BratArray[poz]);
                mb1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                mb2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(500);
                mb1.setPower(0.2);
                mb2.setPower(0.2);
                //SCutie.setPosition(SCutieArray[poz]);
                while (mb1.isBusy() && mb2.isBusy() && !isStopRequested()) {
                    telemetry.addData("pozitia brat", mb1.getCurrentPosition());
                    telemetry.addData("pozitia brat2", mb2.getCurrentPosition());
                    telemetry.update();
                }
            }
        }

        //if (gamepad1.x) // DE SCHIMBAT CU BUTTONREADER
        //{
        //    SCutie.setPosition(SCutie.getPosition() - 0.001);
        //}


    }




    void WeGottaKillOurselves()
    {
        /**
        Hang.readValue();
        if (Hang.wasJustPressed())
        {
            ky5_1.setTargetPosition(500);
            ky5_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ky5_1.setPower(0.5);
            while (ky5_1.isBusy() && isStopRequested())
                ;
        }
        Fall.readValue();
        if (Fall.wasJustPressed())
        {
            ky5_1.setTargetPosition(0);
            ky5_1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ky5_1.setPower(0.5);
        }
         */

        //Hang.readValue();
        //Fall.readValue();
        if (gamepad1.dpad_up)
        {
            ky5_1.setVelocity(-350);
            ky5_2.setVelocity(500);
        }
        else
        if (gamepad1.dpad_down)
        {
            ky5_1.setVelocity(350);
            ky5_2.setVelocity(-500);
        }
        else
        {
            ky5_1.setPower(0);
            ky5_2.setPower(0);
        }

    }

}
