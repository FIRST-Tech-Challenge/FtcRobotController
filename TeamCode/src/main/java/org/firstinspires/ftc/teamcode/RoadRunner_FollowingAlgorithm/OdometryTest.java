package org.firstinspires.ftc.teamcode.RoadRunner_FollowingAlgorithm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test_Pozitie", group="Iterative Opmode")
public class OdometryTest extends LinearOpMode {
    //Constante
    //TODO De masurat:  1. distanta encoder stanga-dreapta
    //                  2. distanta mijloc encoder din mijloc (sper ca encoderul e pus fix pe mijloc)


    private static final double    Track_Width = 1,  //cm
                            Forward_Offset = 1;  //cm
    private static final double       WHEEL_RADIUS = 1; //cm
    private static final double    TICKS_PER_REV = 8192;

    //Variabile

    private double  Target_PosX = 0,
                    Target_PosY = 0,
                    Target_PosAlfa = 0;
    private double  curr_LeftEncPos = 0,
                    curr_RightEncPos = 0,
                    curr_MidEncPos = 0;
    private double  delta_LeftEncPos = 0,
                    delta_RightEncPos = 0,
                    delta_MidEncPos = 0;

    private double  prev_LeftEncPos = 0,
                    prev_RightEncPos = 0,
                    prev_MidEncPos = 0;
    
    private double  Pos_X=0,
                    Pos_Y=0,
                    heading=0;

    private double  heading_pos = 0,
                    OX_pos = 0,
                    OY_pos=0;

    private double  delta_OX=0,
                    delta_OY=0;
    //Encodere
    private Encoder leftEnc = null,
                    rightEnc = null,
                    midEnc = null;

    //Motoare
    private DcMotor LBM = null,
                    RBM = null,
                    LFM = null,
                    RFM = null;

    
    @Override
    public void runOpMode() throws InterruptedException {
        //Encodere de odometrie
        leftEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"));
        rightEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder"));
        midEnc = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder"));

        //Motoare
        LBM  = hardwareMap.get(DcMotor.class, "LBM");
        RBM = hardwareMap.get(DcMotor.class, "RBM");
        LFM  = hardwareMap.get(DcMotor.class, "LFM");
        RFM = hardwareMap.get(DcMotor.class, "RFM");

        //TODO Reverse la encoderele de la rotile de odometrie
        //Reverse la encoderele de la rotile de odometrie
        leftEnc.setDirection(Encoder.Direction.FORWARD);
        rightEnc.setDirection(Encoder.Direction.FORWARD);
        midEnc.setDirection(Encoder.Direction.FORWARD);

        //Reverse la motoare
        LFM.setDirection(DcMotor.Direction.FORWARD);
        LBM.setDirection(DcMotor.Direction.FORWARD);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.REVERSE);

        //START
        waitForStart();

        //LOOP
        while(!isStopRequested())
        {
            updateOdometryPos();
            if(gamepad1.a)
                Target_PosX+=10;
            else if(gamepad1.b)
                Target_PosY+=10;
            else if(gamepad1.x)
                Target_PosX-=10;
            else if(gamepad1.y)
                Target_PosY-=10;
            setMotorPower(0.3);
            telemetry.addData("Pozitie pe OX fata de start:",Pos_X);
            telemetry.addData("Pozitie pe OY fata de start:",Pos_Y);
            telemetry.addData("Unghi rotire:",heading);
            telemetry.update();
        }
    }
    private void citireEncodere()
    {
        int LeftPos = leftEnc.getCurrentPosition();
        int RightPos = rightEnc.getCurrentPosition();
        int MidPos = midEnc.getCurrentPosition();

        curr_LeftEncPos = encoderTicksToCms(LeftPos);
        curr_RightEncPos = encoderTicksToCms(RightPos);
        curr_MidEncPos = encoderTicksToCms(MidPos);
    }
    public void updateOdometryPos()
    {
        citireEncodere();

        //Delta pozitia_actuala - pozitia_trecuta
        delta_LeftEncPos = curr_LeftEncPos - prev_LeftEncPos;
        delta_RightEncPos = curr_RightEncPos - prev_RightEncPos;
        delta_MidEncPos = curr_MidEncPos - prev_MidEncPos;

        //Calcul deplasare
        heading_pos = (delta_LeftEncPos - delta_RightEncPos) / Track_Width;
        OX_pos = (delta_RightEncPos + delta_LeftEncPos) / 2;
        OY_pos = delta_MidEncPos - heading_pos * Forward_Offset;

        //Delta deplasare pe axe
        delta_OX = OX_pos * Math.cos(heading) - OY_pos * Math.sin(heading);
        delta_OY = OX_pos * Math.sin(heading) + OY_pos * Math.cos(heading);

        //Actualizare pozitie axe+heading
        Pos_X += delta_OX;
        Pos_Y += delta_OY;
        heading += heading_pos;

        //Trecut = prezent
        prev_LeftEncPos = curr_LeftEncPos;
        prev_RightEncPos = curr_RightEncPos;
        prev_MidEncPos = curr_MidEncPos;
    }
    private static double encoderTicksToCms(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }
    private void setMotorPower(double pow)
    {
        if(Pos_Y<Target_PosY+5||Target_PosY-5<Pos_Y) {
            if(Pos_Y<Target_PosY)
                pow=-pow;
            LFM.setPower(pow);
            LBM.setPower(pow);
            RBM.setPower(pow);
            RFM.setPower(pow);
        }
    }
}
