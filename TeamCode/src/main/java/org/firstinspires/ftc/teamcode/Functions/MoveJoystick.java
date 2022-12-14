package org.firstinspires.ftc.teamcode.Functions;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Functions.MV.MVVariables;

public class MoveJoystick {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private int currentDirection;

    void Init(){
        try {
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            currentDirection = 0;
        }
        catch(NullPointerException e){
        }
    }

    public MoveJoystick(DcMotor _LMF, DcMotor _RMF, DcMotor _LMB, DcMotor _RMB){
        leftMotor = _LMF;
        rightMotor = _RMF;
        leftMotorBack = _LMB;
        rightMotorBack = _RMB;
        Init();
    }

    public void MoveJoystickRaw(int direction, double power){
        try{
            currentDirection =direction;
            switch(direction){
                case 1:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(-power);
                    rightMotorBack.setPower(power);
                    rightMotor.setPower(power);
                    break;
                case 2:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(power);
                    rightMotorBack.setPower(-power);
                    rightMotor.setPower(-power);
                    break;
                case 3:
                    leftMotor.setPower(power);
                    leftMotorBack.setPower(-power);
                    rightMotorBack.setPower(-power);
                    rightMotor.setPower(power);
                    break;
                case 4:
                    leftMotor.setPower(-power);
                    leftMotorBack.setPower(power);
                    rightMotorBack.setPower(power);
                    rightMotor.setPower(-power);
                    break;
            }
        }
        catch(NullPointerException e){
            // telemetry.addData("Status Move", "ERROR NAME:"+e.ToString());
        }
    }

    public void MoveJoystickStop(){
        try{
            currentDirection =0;
            leftMotor.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);
            rightMotor.setPower(0);
        }
        catch(NullPointerException e){
            // telemetry.addData("Status Move", "ERROR NAME:"+e.ToString());
        }
    }



}
