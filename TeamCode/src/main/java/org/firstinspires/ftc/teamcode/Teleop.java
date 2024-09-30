package org.firstinspires.ftc.teamcode;

import com.parshwa.drive.Drive;
import com.parshwa.drive.DriveModes;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "Parshwa\'s teleop with his custom module")
public class Teleop extends LinearOpMode {
    private Drive driver = new Drive();
    private double SPED = 0;
    private IMU imu;
    private RevHubOrientationOnRobot orientation;
    private boolean ALINMODE = false;
    private boolean completed = false;
    private boolean decided = false;
    private RevBlinkinLedDriver light;
    private Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        light = hardwareMap.get(RevBlinkinLedDriver.class,"lights");
        light.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
        limelight = hardwareMap.get(Limelight3A.class, "cam");
        /*while(!decided){
            if(gamepad1.a){
                orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
            }
            if(gamepad1.b){
                orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
            }
            if(gamepad1.x){
                orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);
            }
        }*/
        orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
        imu = hardwareMap.get(IMU.class, "imu");
        if (orientation == null) {
            orientation = new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.DOWN,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        }
        imu.initialize(new IMU.Parameters(orientation));
        driver.change(imu);
        driver.change("RFM","RBM","LFM","LBM");
        driver.change(DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.FORWARD,
                DcMotorSimple.Direction.REVERSE,
                DcMotorSimple.Direction.REVERSE);
        while(!isStopRequested()){
            if(gamepad1.a){
                driver.init(hardwareMap,telemetry, DriveModes.MecanumFeildOriented);
                break;
            }
            else if(gamepad1.b){
                driver.init(hardwareMap,telemetry, DriveModes.MecanumRobotOriented);
                break;
            }
        }
        telemetry.addLine("initilized");
        telemetry.update();
        waitForStart();

        while (!isStopRequested()){
            SPED = gamepad1.right_trigger / 2.0;
            if(gamepad1.right_bumper){
                SPED = SPED * 2.0;
            }
            if(gamepad1.a){
                ALINMODE = true;
                completed = false;
            }
            driver.move(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad1.right_stick_x,SPED);
            while(ALINMODE && !isStopRequested()){

                limelight.pipelineSwitch(0);
                limelight.reloadPipeline();
                limelight.start();
                LLResult result = limelight.getLatestResult();
                if(gamepad1.x) {
                    telemetry.addLine(result.toString());
                }else{
                    telemetry.addLine(limelight.getStatus().toString());
                }
                telemetry.update();
                if(completed || gamepad1.b){
                    ALINMODE = false;
                    limelight.stop();
                }
            }
        }
    }
}
