package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstsForTeleskope;

public class AutoMethodsTime implements ConstsForTeleskope {
    private OpMode op;

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightF, rightB, leftB, leftF, teleskopUpStanding;

    private Servo _20kg, povorot, kleshni, korzina;

    public void initC(OpMode op){
        //Инициализация
        rightF = op.hardwareMap.get(DcMotor.class, "rightF");//m1
        rightB = op.hardwareMap.get(DcMotor.class, "rightB");//m2
        leftB = op.hardwareMap.get(DcMotor.class, "leftB");//m3
        leftF = op.hardwareMap.get(DcMotor.class, "leftF");//m4

        teleskopUpStanding = op.hardwareMap.get(DcMotor.class, "teleskop");

        _20kg = op.hardwareMap.get(Servo.class, "20kg");
        povorot = op.hardwareMap.get(Servo.class, "povorot");
        kleshni = op.hardwareMap.get(Servo.class, "kleshni");
        korzina = op.hardwareMap.get(Servo.class, "korzina");

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

    public void forward (double time, double max_Power){
        double max = Math.abs(max_Power);

        while(runtime.seconds() < time) {
            rightF.setPower(max);
            rightB.setPower(max);
            leftB.setPower(max);
            leftF.setPower(max);
        }
    }

    public void back(double time, double max_Power){
        double max = -Math.abs(max_Power);

        while(runtime.seconds() < time) {
            rightF.setPower(max);
            rightB.setPower(max);
            leftB.setPower(max);
            leftF.setPower(max);
        }
    }

    public void left(double time, double max_Power){
        double max = Math.abs(max_Power);

        while(runtime.seconds() < time) {
            rightF.setPower(max);
            leftB.setPower(max);

            rightB.setPower(-max);
            leftF.setPower(-max);
        }
    }

    public void right(double time, double max_Power){
        double max = Math.abs(max_Power);

        while(runtime.seconds() < time) {
            rightF.setPower(-max);
            leftB.setPower(-max);

            rightB.setPower(max);
            leftF.setPower(max);
        }
    }

    public void rotateClockWise(double time, double max_Power){
        double max = Math.abs(max_Power);

        while(runtime.seconds() < time) {
            leftB.setPower(max);
            leftF.setPower(max);

            rightB.setPower(-max);
            rightF.setPower(-max);
        }
    }

    public void rotateCounterClockwise(double time, double max_Power){
        double max = Math.abs(max_Power);

        while(runtime.seconds() < time) {
            leftB.setPower(-max);
            leftF.setPower(-max);

            rightB.setPower(max);
            rightF.setPower(max);
        }
    }

    public Servo get_20kg() {
        return _20kg;
    }

    public Servo getKleshni() {
        return kleshni;
    }

    public Servo getKorzina() {
        return korzina;
    }

    public Servo getPovorot() {
        return povorot;
    }
}
