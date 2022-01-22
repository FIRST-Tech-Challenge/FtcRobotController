package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.mason;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.FieldConstant;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive_Chassis2;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

@TeleOp(name = "TEST", group = "Competition")
@Disabled
public class Test extends LinearOpMode {

    BNO055IMU imu;
    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;

    @Override
    public void runOpMode() throws InterruptedException{


        //Traj
        SampleMecanumDrive_Chassis2 drive = new SampleMecanumDrive_Chassis2(hardwareMap);
        Pose2d startPose = FieldConstant.BLUE_DUCK_STARTING_POSE;
        drive.setPoseEstimate(startPose);

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);
        double speed = 0.5;


        waitForStart();

        boolean releasedX1 = true;
        boolean releasedY1 = true;
        boolean releasedA1 = true;
        boolean releasedB1 = true;

        double LFPower = 0;
        double RFPower = 0;
        double LBPower = 0;
        double RBPower = 0;



        while (opModeIsActive()) {

            drive.update();

            // Retrieve your pose
            Pose2d myPose = drive.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            double straight = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if(gamepad1.dpad_up){
                if(releasedX1) {
                    LFPower = 1;
                }
                releasedX1 = false;
            } else if(!releasedX1){
                LFPower = 0;
                releasedX1 = true;

            }

            if(gamepad1.dpad_down){
                if(releasedY1) {
                    LBPower = 1;
                }
                releasedY1 = false;
            } else if(!releasedY1){
                LBPower = 0;
                releasedY1 = true;

            }

            if(gamepad1.dpad_left){
                if(releasedA1) {

                    RFPower = 1;
                }
                releasedA1 = false;
            } else if(!releasedA1){
                RFPower = 0;
                releasedA1 = true;

            }

            if(gamepad1.dpad_right){
                if(releasedB1) {
                    RBPower = 1;
                }
                releasedB1 = false;
            } else if(!releasedB1){
                RBPower = 0;
                releasedB1 = true;

            }


            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);
            }
        }

    }
