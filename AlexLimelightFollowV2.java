package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Alex's limelight that is cool")
public class AlexLimelightFollowV2 extends LinearOpMode{
    double tx = 0;
    double ta = 0;
    double desiredDistance = -2;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor BR = hardwareMap.dcMotor.get("BR");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.setPollRateHz(100);
        limelight.start();
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Robot is ready!", "Skibidi Toliet Rizz!");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx(); // How far left or right the target is (degrees)
                ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target A", ta);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
                tx = 0;
                ta = 0;
            }
            telemetry.update();
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
            if(result.isValid() == false) {
                movepower = 0;
                turnpower = 0;
            }
            movepower = -movepower;
            FL.setPower(turnpower+movepower);
            FR.setPower(movepower-turnpower);
            BL.setPower(turnpower+movepower);
            BR.setPower(movepower-turnpower);



        }
    }
}