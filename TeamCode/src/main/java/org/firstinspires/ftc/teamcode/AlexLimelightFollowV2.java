package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Alex's limelight that is cool")
public class AlexLimelightFollowV2 extends LinearOpMode{
    double tx = 0; // must define it as double here to make it global
    double ta = 0;
    double ty = 0;
    double slidespower = 0;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FL = hardwareMap.dcMotor.get("FL"); //init motors
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor BR = hardwareMap.dcMotor.get("BR");
        DcMotor slides = hardwareMap.dcMotor.get("slides");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // get the pipeline for limelight
        limelight.setPollRateHz(100);
        limelight.start(); // starts, REQUIRED FOR LIMELIGHT!
        FL.setDirection(DcMotorSimple.Direction.REVERSE); // regular drivetrain reverse
        BL.setDirection(DcMotorSimple.Direction.REVERSE); // regular drivetrain reverse
        slides.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoder for driving
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // sets mode to run without encoder because we don't need to caluclate volocity
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!"); //hehe
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) { // if the limelight sees a apriltag
                tx = result.getTx(); // How far left or right the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)
                ty = result.getTy();
                telemetry.addData("Target X", tx);
                telemetry.addData("Target A", ta);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
                tx = 0;
                ta = 0;
                ty = 0;
            }

            double tuningconstant = 0.02;
            double turnpower = tx * tuningconstant;
            if(turnpower >= 0.7) {
               turnpower = 0.7;
            }
            if(Math.abs(tx) <= 2) {
                turnpower = 0;
            }
            double desiredta = 10;
            double movepower = (desiredta - ta) * tuningconstant;
            if(movepower >= 0.7) {
                movepower = 0.7;
            }
            if(movepower <= -0.7) {
                movepower = -0.7;
            }
            double slidestuningconstant = 0.028;
            if(Math.abs(ty) >= 2.5) {
                slidespower = ty * slidestuningconstant;
            } else {
                slidespower = 0;
            }

            if(slidespower <= 0.4 && slidespower > 0) {
                slidespower = 0.4;
            }
            if(slidespower >= 0.8 && slidespower > 0) {
                slidespower = 0.8;
            }
            if(result.isValid() == false) {
                movepower = 0;
                turnpower = 0;
                slidespower = 0;
            }
            if(-slides.getCurrentPosition() < -2988 && slidespower > 0) {
                slidespower = 0;
            }
            if(-slides.getCurrentPosition() > -5 && slidespower < 0) {
                slidespower = 0;
            }
            telemetry.addData("Slides Current Position", slides.getCurrentPosition());
            telemetry.update();
            movepower = -movepower;
            FL.setPower(turnpower+movepower);
            FR.setPower(movepower-turnpower);
            BL.setPower(turnpower+movepower);
            BR.setPower(movepower-turnpower);
            slides.setPower(slidespower);



        }
    }
}