package org.firstinspires.ftc.teamcode.TeleOps;

import static com.qualcomm.robotcore.util.Range.clip;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp (name = "PIDFArm")
public class PIDFArm extends OpMode {
    private PIDController controller;
    private PIDController controller2;

    public static  double p = 0, i = 0 , d = 0;
    public static  double p1 = 0, i1 = 0 , d1 = 0;//p1=0.0009,d1=0.00001,f=0.01
    public  static double f=0;

    public static int target =0;
    public static  double speed = 0;

    private final double ticks_in_degree = 537.7/360;

    private DcMotorEx deposit_Slide_Left;
    private DcMotorEx deposit_Slide_Right;
    private DcMotorEx test_motor;
    /**
    private ColorSensor colorSensor;

    private int redValue;
    private int greenValue;
    private int blueValue;

     **/
    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        controller2 = new PIDController(p1, i1, d1);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        deposit_Slide_Left = hardwareMap.get(DcMotorEx.class, "VS_Left_Motor");
        //deposit_Slide_Right = hardwareMap.get(DcMotorEx.class, "VS_Right_Motor");
        test_motor = hardwareMap.get(DcMotorEx.class, "Test_Motor");

        deposit_Slide_Left.setDirection(DcMotorSimple.Direction.REVERSE);
        test_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        //deposit_Slide_Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //deposit_Slide_Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //deposit_Slide_Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //deposit_Slide_Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //deposit_Slide_Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //deposit_Slide_Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //test_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        controller2.setPID(p1,i1,d1);
        int armPos_left = deposit_Slide_Left.getCurrentPosition();
        //int armPos_right = deposit_Slide_Right.getCurrentPosition();
        int armPos = test_motor.getCurrentPosition();

        double pid_left = controller2.calculate(armPos_left, target);
        double pid = controller.calculate(armPos, target);
        //double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;
        double ff = 1 * f;

        double power_left = pid_left + ff;
        double power = pid + ff;

        test_motor.setPower(power);
        deposit_Slide_Left.setPower(power_left);

        //deposit_Slide_Left.setTargetPosition(target);
        //deposit_Slide_Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //deposit_Slide_Left.setPower(speed);
        //deposit_Slide_Right.setTargetPosition(target);
        //deposit_Slide_Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //deposit_Slide_Right.setPower(speed);

        telemetry.addData("pos_left ", armPos_left);
        //telemetry.addData("pos_right ", armPos_right);
        telemetry.addData("pos ", armPos);
        telemetry.addData("target int ", target);
        //telemetry.addData("Color Sensor ", colorSensor.red());

        telemetry.update();
    }

}
