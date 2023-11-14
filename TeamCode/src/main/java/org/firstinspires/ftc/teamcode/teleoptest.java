package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="test", group="Linear Opmode")

public class teleoptest extends GlobalScope2023 {
    DcMotorEx motorb,mb1,mb2;
    CRServo servorot;
    Servo servoCS;
    Servo servoCD;
    public void inits()
    {
        mb1=hardwareMap.get(DcMotorEx.class,"mb1");
        mb2=hardwareMap.get(DcMotorEx.class,"mb2");
        mb1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mb2.setDirection(DcMotorSimple.Direction.REVERSE);
        motorb=hardwareMap.get(DcMotorEx.class,"motorb");
        motorb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servorot=hardwareMap.get(CRServo.class, "servorot");
        servoCS=hardwareMap.get(Servo.class, "servocs");
        servoCD=hardwareMap.get(Servo.class, "servocd");
        motorb.setTargetPositionTolerance(0);
    }

    private ElapsedTime runtime = new ElapsedTime();

    public void runOpMode() {
        inits();
        waitForStart();
        //double posstartservorot=servorot.getPosition();
        servoCS.setPosition(0);
        servoCD.setPosition(0);
        double posservoCS=servoCS.getPosition();
        double posservoCD=servoCD.getPosition();
        int motorbpos= motorb.getCurrentPosition();
        while (opModeIsActive()) {
            motorb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            while (gamepad1.a)
                servorot.setPower(1);
            while (gamepad1.b)
                servorot.setPower(-1);
            servorot.setPower(0);
            if (gamepad1.dpad_up) {
                motorb.setTargetPosition(motorb.getCurrentPosition() + 10);
                motorb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorb.setPower(0.8);
                while (motorb.isBusy()) ;
                motorb.setPower(0);
            }
            if (gamepad1.dpad_down) {
                motorb.setTargetPosition(motorb.getCurrentPosition() - 10);
                motorb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorb.setPower(0.8);
                while (motorb.isBusy()) ;
                motorb.setPower(0);
            }
            if (gamepad1.left_bumper) {
                servoCS.setPosition(posservoCS + 0.01);
                servoCS.setDirection(Servo.Direction.FORWARD);
            }
            if (gamepad1.left_trigger > 0.1) {
                servoCS.setPosition(posservoCS - 0.01);
                servoCS.setDirection(Servo.Direction.REVERSE);
            }
            if (gamepad1.right_bumper){
                servoCD.setPosition(posservoCD + 0.01);
                servoCD.setDirection(Servo.Direction.REVERSE);
            }
            if (gamepad1.right_trigger > 0.1) {
                servoCD.setPosition(posservoCD - 0.01);
                servoCD.setDirection(Servo.Direction.FORWARD);
            }
            //servo.interrupt();
            while(gamepad1.left_stick_y>0.1)
            {
                mb1.setPower(0.5);
                mb2.setPower(0.5);
            }
            while(gamepad1.left_stick_y<-0.1)
            {
                mb1.setPower(-0.5);
                mb2.setPower(- 0.5);
            }
            mb1.setPower(0);
            mb2.setPower(0);
            telemetry.addData("poz encoder ", motorb.getCurrentPosition());
            telemetry.update();
        }
    }
}