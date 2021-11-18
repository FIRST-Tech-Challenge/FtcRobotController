package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Basic: Iterative opMode", group = "Iterative Opmode")
public class MoveTest extends OpMode {
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private ElapsedTime runtime = new ElapsedTime();

    public void init(){
      telemetry.addData("Status", "Initialized");
      leftDrive = hardwareMap.get(DcMotor.class, "leftMotor");
      rightDrive = hardwareMap.get(DcMotor.class,"rightMotor");
      leftDrive.setDirection(DcMotor.Direction.FORWARD);
      rightDrive.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void init_loop(){

    }
    @Override
    public void start(){
        runtime.reset();
    }
    @Override
    public void loop(){
        double LeftPower = gamepad1.left_stick_y;
        double RightPower = gamepad1.right_stick_y;
        if(gamepad1.a){
            leftDrive.setPower(LeftPower*1);
            rightDrive.setPower(RightPower*1);
        }
        else{
            leftDrive.setPower(LeftPower*0.5);
            rightDrive.setPower(RightPower*0.5);
        }
        telemetry.addData("Status","Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f)", LeftPower, RightPower);
    }
    @Override
    public void stop(){

    }
}
