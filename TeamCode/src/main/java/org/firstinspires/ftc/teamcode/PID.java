package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "PID")
public class PID extends BasicOpMode_Linear {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime timer = new ElapsedTime();

    double integralSum = 0;
    double Kp = 0;
    double Ki = 0;
    double Kd = 0;
    double Kf = 0;
    private double lastError = 0;


    @Override
    public void runOpMode(){

        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()){
            double lfpower = PIDControl(100,leftFrontDrive .getCurrentPosition());
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftFrontDrive.setPower(lfpower);

            double lbpower = PIDControl(100,leftBackDrive .getCurrentPosition());
            leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            leftBackDrive.setPower(lbpower);

            double rfpower = PIDControl(100,rightFrontDrive .getCurrentPosition());
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightFrontDrive.setPower(rfpower);

            double rbpower = PIDControl(100,rightBackDrive .getCurrentPosition());
            rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            rightBackDrive.setPower(rbpower);
        }


    }
    public double PIDControl(double reference,double state){
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative  = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);

        return output;
    }
}







