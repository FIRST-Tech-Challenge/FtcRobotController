package org.firstinspires.ftc.teamcode.Robot;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Drive{
    HardwareMap hMap = null;
    DcMotor dcMotorA = null;
    DcMotor dcMotorB = null;
    DcMotor dcMotorC = null;
    DcMotor dcMotorD = null;
    DcMotor dcMotorE = null;
    DcMotor dcMotorF = null;
    ElapsedTime runTime = new ElapsedTime();
    static final int LEFT_EXTENDER_ENDSTOP = 1695;
    static final int RIGHT_EXTENDER_ENDSTOP = 1695;

    //hi
    public Drive(HardwareMap hardwareMap){
                hMap = hardwareMap;
                dcMotorA = hMap.get(DcMotorEx.class, "leftFront");
                dcMotorB = hMap.get(DcMotorEx.class, "leftBack");
                dcMotorC = hMap.get(DcMotorEx.class, "rightBack");
                dcMotorD = hMap.get(DcMotorEx.class, "rightFront");
                dcMotorC.setDirection(DcMotorSimple.Direction.REVERSE);
                dcMotorD.setDirection(DcMotorSimple.Direction.REVERSE);
                dcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                dcMotorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dcMotorB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dcMotorC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dcMotorD.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                dcMotorE = hardwareMap.get(DcMotor.class, "leftExtender");
                dcMotorF = hardwareMap.get(DcMotor.class, "rightExtender");
                runTime.reset();
            }

   public void driveTiles(float Tiles) {
       dcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorA.setTargetPosition((int) (-993 * Tiles));
       dcMotorB.setTargetPosition((int) (-1044 * Tiles));
       dcMotorC.setTargetPosition((int) (-1037 * Tiles));
       dcMotorD.setTargetPosition((int) (-1061 * Tiles));
       dcMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorA.setPower(.75);
       dcMotorB.setPower(.75);
       dcMotorC.setPower(.75);
       dcMotorD.setPower(.75);
       sleep(2000);
   }
   public void setRotateDegrees(double Deg) {
       dcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       dcMotorA.setTargetPosition((int) (9.5 * Deg));
       dcMotorB.setTargetPosition((int) (10.36 * Deg));
       dcMotorC.setTargetPosition((int) (-11.3 * Deg));
       dcMotorD.setTargetPosition((int) (-10.6 * Deg));
       dcMotorA.setPower(.75);
       dcMotorB.setPower(.75);
       dcMotorC.setPower(.75);
       dcMotorD.setPower(.75);
       dcMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       dcMotorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
       sleep(2000);
    }
    public void strafeTiles (float toBeStrafed) {
        dcMotorA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcMotorA.setTargetPosition((int) (890 * toBeStrafed));
        dcMotorB.setTargetPosition((int) (-789 * toBeStrafed));
        dcMotorC.setTargetPosition((int) (909 * toBeStrafed));
        dcMotorD.setTargetPosition((int) (-772 * toBeStrafed));
        dcMotorA.setPower(.75);
        dcMotorB.setPower(.75);
        dcMotorC.setPower(.75);
        dcMotorD.setPower(.75);
        dcMotorA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotorB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotorC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcMotorD.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
    }

    public String gatherMotorPos(){
        return "dcMotorA Position " + dcMotorA.getCurrentPosition() +
                "\ndcMotorB Position " + dcMotorB.getCurrentPosition() +
                "\ndcMotorC Position " + dcMotorC.getCurrentPosition() +
                "\nDcMotorD Position " + dcMotorD.getCurrentPosition();


    }


}

