package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Alex's limelight that is cool")
public class AlexLimelightFollowV2 extends LinearOpMode{
    double tx = 0; // must define it as double here to make it global
    double ta = 0;
    double ty = 0;
    double slidespower = 0;
    boolean slidesmove = true;
    double targetposition;
    private Limelight3A limelight;
    double slidestargetposition = 0;

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
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //Resets encoder for driving
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // sets mode to run without encoder because we don't need to caluclate volocity
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide slide = new Slide();
        slide.slidesinit(hardwareMap);
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


            slidestargetposition = (ty*20.6923) + slides.getCurrentPosition();
            if(result.isValid() == false) {
                movepower = 0;
                turnpower = 0;
            }
            if(-slides.getCurrentPosition() < -2988 && slidestargetposition > slides.getCurrentPosition()) {
                slidesmove = false;
            }
            if(-slides.getCurrentPosition() > -5 && slidestargetposition < slides.getCurrentPosition()) {
                slidesmove = false;
            }
            telemetry.addData("Slides Current Position", slides.getCurrentPosition());
            telemetry.update();
            movepower = -movepower;
            FL.setPower(turnpower+movepower);
            FR.setPower(movepower-turnpower);
            BL.setPower(turnpower+movepower);
            BR.setPower(movepower-turnpower);
           if (slidesmove == true && Math.abs(ty) > 1.5) {
                slide.slidego(slidestargetposition);
            }


        }
    }
}