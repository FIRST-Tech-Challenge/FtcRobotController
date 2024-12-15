package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.

@Autonomous
import org.firstinspires.ftc.teamcode.auto.OdometryMotor;

public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        ElapsedTime runtime = new ElapsedTime();
        //define robots attachments to work
        DcMotor frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        DcMotor frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor backleftDrive = hardwareMap.get(DcMotor.class, "backleft");
        OdometryMotor straight = new OdometryMotor("sideways", OdometryMotor.WHEELTYPE.MM, 48, OdometryMotor.TYPE.TICKPERREV, 2000 );

        Servo outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        Servo intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        Servo intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        straight.setMotor(hardwareMap.get(DcMotorEx.class, straight.motorname));

        IMU imu = hardwareMap.get(IMU.class,"imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

//        final double start = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        imu.resetYaw();
        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        boolean forward = true;
        boolean where = false;
        double pos ;


        double ticks = 10/ 0.0029688;
        int position = straight.getMotor().getCurrentPosition() + (int)ticks;// -((int)((1.282*ticks)+336.6)-(int)ticks);
        // straight.getMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        straight.getMotor().setTargetPosition(position);
        straight.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // int posC = straight.getMotor().getCurrentPosition();
        while(!hadssedThes(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            frontrightDrive.setPower(-0.1);
            frontleftDrive.setPower(-0.1);
            backleftDrive.setPower(-0.1);
            backrightDrive.setPower(-0.1);


        }
        frontleftDrive.setPower(-0.0);
        frontrightDrive.setPower(-0.0);
        backleftDrive.setPower(-0.0);
        backrightDrive.setPower(-0.0);        while(true){
            // telemetry.addData("tar", straight.getMotor().getTargetPosition());
            // telemetry.addData("cur", straight.getMotor().getCurrentPosition());
            // telemetry.update();
        }
        // while(opModeIsActive()){
        //     pos=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //     frontleftDrive.setPower(0.2);
        //     backleftDrive.setPower(0.2);
        //     frontrightDrive.setPower(-0.2);
        //     backrightDrive.setPower(-0.2);
        //     if ((pos < 5 && pos > -5) && forward==false){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=true;
        //         where=true;
        //     }
        //     if ((pos < -175 &&  pos > 175) && forward==true){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=false;
        //         where=true;
        //     }
        //     telemetry.addData("x", pos);
        //     telemetry.addData("forward", forward);
        //     telemetry.addData("cur",  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //     telemetry.addData("where", where);
        //     telemetry.update();
        // }
    }

    private void wait(double sec, ElapsedTime runtime){
        runtime.reset();
        while(runtime.seconds() < sec){
        }
    }

    private boolean hadssedThes(int input, int target){
        if(target > 0){
            return input >= target;
        } else {
            return input <= target;
        }
    }
}
