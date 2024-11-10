package org.firstinspires.ftc.teamcode.COMPETITIONCODE;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.parshwa.drive.tele.Drive;
import com.parshwa.drive.tele.DriveModes;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.COMPETITIONCODE.data.SliderManger;
import org.firstinspires.ftc.teamcode.COMPETITIONCODE.data.servoManger;

import java.io.File;

@TeleOp(name = "teleop")
public class Teleop extends LinearOpMode {
    private servoManger clawServo = new servoManger();
    private servoManger clawRotateServo = new servoManger();
    private servoManger clawRotateServo2 = new servoManger();

    private Drive driver = new Drive();
    private double SPED = 0;
    private IMU imu;
    private RevHubOrientationOnRobot orientation;
    private SliderManger SM = new SliderManger();
    private DcMotor sc, sr;
    @Override
    public void runOpMode() throws InterruptedException {
        RevTouchSensor touchSensor = hardwareMap.get(RevTouchSensor.class, "touch");
        clawServo.init(hardwareMap, "cs");
        clawRotateServo.init(hardwareMap, "crs");
        clawRotateServo2.init(hardwareMap, "crs2");
        sc = hardwareMap.dcMotor.get("sc");
        sr = hardwareMap.dcMotor.get("sr");
        sc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        SM.init(sc,sr);

        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE);
        driver.init(hardwareMap,telemetry, DriveModes.MecanumFeildOriented);
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();
        clawServo.setServoPosition(0.0);
        clawRotateServo.setServoPosition(0.7);
        clawRotateServo2.setServoPosition(0.55);
        while (!isStopRequested()){
            //driver1
            SPED = gamepad1.right_trigger;
            if(gamepad1.right_bumper){
                SPED /= 2.0;
            }
            driver.move(gamepad1.left_stick_y,-gamepad1.left_stick_x,gamepad1.right_stick_x,SPED);
            //driver2
            SM.move(gamepad2.left_stick_y + 0.01  > 1.0 && sc.getCurrentPosition() < 20 && sr.getCurrentPosition() > 600 ? gamepad2.left_stick_y : gamepad2.left_stick_y + 0.01);
            telemetry.addLine("Slide Pos:" + String.valueOf(sc.getCurrentPosition()));
            if(-gamepad2.right_stick_y <= -0.3){
                SM.setPos(0);
            }
            if(gamepad2.left_trigger > 0.3){
                clawRotateServo.setServoPosition(0.4);
                safeWaitSeconds(0.5);
                int RotateTarget = 675;
                while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
                    SM.setPos(RotateTarget);
                }
                int target = -3000;
                while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
                    SM.setPos2(target);
                }
                clawRotateServo.setServoPosition(0.45);
                safeWaitSeconds(0.5);
                clawServo.setServoPosition(0.39);
                safeWaitSeconds(0.5);
                clawRotateServo.setServoPosition(0.05);
                safeWaitSeconds(0.5);
                target = 0;
                while(!isStopRequested() && !(Math.abs(target-sc.getCurrentPosition()) <= 10)){
                    SM.setPos2(target);
                }
                RotateTarget = 0;
                while(!isStopRequested() && !(Math.abs(RotateTarget-sr.getCurrentPosition()) <= 10)){
                    SM.setPos(RotateTarget);
                }
                sc.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if(-gamepad2.right_stick_y >= 0.3){
                SM.setPos(675);
            }
            if(touchSensor.isPressed()){
                SM.reset();
            }
            telemetry.addLine(String.valueOf(sr.getCurrentPosition()));
            if(gamepad2.b) {
                clawRotateServo.setServoPosition(0.1);
            }
            if(gamepad2.a) {
                clawRotateServo.setServoPosition(0.05);
            }
            if(gamepad2.y) {
                clawRotateServo.setServoPosition(0.4);
            }
            if(gamepad2.x) {
                clawRotateServo.setServoPosition(0.6);
            }
            if(gamepad2.right_bumper){
                clawServo.setServoPosition(0.39);
            }
            if(gamepad2.left_bumper){
                clawServo.setServoPosition(0.0);
            }
            if(gamepad2.dpad_left){
                clawRotateServo2.setServoPosition(0.8);
            }else if(gamepad2.dpad_right){
                clawRotateServo2.setServoPosition(0.3);
            }else if(gamepad2.dpad_up){
                clawRotateServo2.setServoPosition(0.1);
            }else if(gamepad2.dpad_down){
                clawRotateServo2.setServoPosition(0.9);
            }else{
                clawRotateServo2.setServoPosition(0.5);
            }
            //SM.setPos2(sc.getCurrentPosition());
        }
        File ThreadManger = AppUtil.getInstance().getSettingsFile("ThreadManger.txt");
        ReadWriteFile.writeFile(ThreadManger, "STOP");
    }
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }
}
