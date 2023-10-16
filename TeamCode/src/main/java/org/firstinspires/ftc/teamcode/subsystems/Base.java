package org.firstinspires.ftc.teamcode.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Base {
    static DcMotor leftFront;
    static DcMotor rightFront;
    static DcMotor leftRear;
    static DcMotor rightRear;
    public static void initBase(HardwareMap hardwareMap){
        leftFront = hardwareMap.dcMotor.get("leftFront");//hwmap
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//reset encoders
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//set runmode
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);//reverse left side
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double lf, double lr,double rf,double rr){
        leftFront.setPower(lf);
        leftRear.setPower(lr);
        rightFront.setPower(rf);
        rightRear.setPower(rr);
    }
    public int getLeftfPos(){
        int pos= leftFront.getCurrentPosition();
        return pos;
    }
    public int getLeftrPos(){
        int pos= leftRear.getCurrentPosition();
        return pos;
    }
    public int getRightfPos(){
        int pos= rightFront.getCurrentPosition();
        return pos;
    }
    public int getRightrPos(){
        int pos= rightRear.getCurrentPosition();
        return pos;
    }

}
