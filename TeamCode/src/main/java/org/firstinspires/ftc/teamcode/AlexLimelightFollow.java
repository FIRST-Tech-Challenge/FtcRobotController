package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Alex's limelight that is cool")
public class AlexLimelightFollow extends LinearOpMode{
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
            if(tx > 4) {
                //turnRight(tx degress) // give it a range that is acceptable
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
                double degressforright = tx;
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int ticksforright = (int)Math.round(degressforright*10.9);
                FL.setTargetPosition(ticksforright);
                BL.setTargetPosition(ticksforright);
                FR.setTargetPosition(ticksforright);
                BR.setTargetPosition(ticksforright);
                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FL.setPower(0.4);
                FR.setPower(0.4);
                BL.setPower(0.4);
                BR.setPower(0.4);
                while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()
                ) {}
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(tx < -4) {
                //turnleft(tx degress) // give it a range that is acceptable
                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                double degressforleft = tx;
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                int ticksforleft = (int)Math.round(degressforleft*10.9);
                FL.setTargetPosition(ticksforleft);
                BL.setTargetPosition(ticksforleft);
                FR.setTargetPosition(ticksforleft);
                BR.setTargetPosition(ticksforleft);
                FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FL.setPower(0.4);
                FR.setPower(0.4);
                BL.setPower(0.4);
                BR.setPower(0.4);

                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(ta > 4.5) {
                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FL.setPower(0.4);
                FR.setPower(0.4);
                BL.setPower(0.4);
                BR.setPower(0.4);
                while (ta < 3.5) {
                    if(result != null && result.isValid()) {
                        tx = result.getTx();   // update tx continuously
                        ta = result.getTa();
                    } else {
                        tx = 0;
                        ta = 0;// or whatever default
                    }
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target A", ta);
                    telemetry.update();
                    sleep(10);
                }
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            if(ta < 3.5 && ta > 0.5){
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FL.setPower(0.4);
                FR.setPower(0.4);
                BL.setPower(0.4);
                BR.setPower(0.4);
                while (ta > 4.5) {
                    if(result != null && result.isValid()) {
                        tx = result.getTx();   // update tx continuously
                        ta = result.getTa();
                    } else {
                        tx = 0; // or whatever default
                        ta = 0;
                    }
                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target A", ta);
                    telemetry.update();
                    sleep(10);
                }
                FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                FR.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);
                BR.setDirection(DcMotorSimple.Direction.REVERSE);




            }



        }
    }
}