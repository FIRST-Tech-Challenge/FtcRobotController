package org.firstinspires.ftc.teamcode.fall2021;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mason_wu.Mecanum_TeleOp;

import java.util.ArrayList;


@TeleOp(name = "Mecanum TeleOp Test", group = "Linear Opmode")
public class Mecanum_TeleOp_Test extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private ArrayList<Double[]> speedList = new ArrayList<Double[]>();
    private ElapsedTime runtime = new ElapsedTime();

    double rotate = 0;
    double speed = 0.5;
    boolean reverse = false;

    public Mecanum_TeleOp_Test() {

    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF  = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB  = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        double LFPower;
        double RFPower;
        double LBPower;
        double RBPower;

        waitForStart();

        boolean releasedRightBumper = true;
        boolean releasedLeftBumper = true;
        boolean releasedGamePad1 = true;

        boolean toggleGamePad1 = true;

        while (opModeIsActive()) {
            runtime.reset();

            double drive = -gamepad1.left_stick_y;
            double strafe  = -gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if(gamepad1.right_bumper) {
                if(releasedRightBumper && releasedLeftBumper) {
                    increaseSpeed(0.05);
                    releasedRightBumper = false;
                }
            } else {
                releasedRightBumper = true;
            }

            if(gamepad1.left_bumper){
                if(releasedRightBumper && releasedLeftBumper) {
                    decreaseSpeed(0.05);
                    releasedLeftBumper = false;
                }
            } else {
                releasedLeftBumper = true;
            }

            if (gamepad1.x) {
                if (releasedGamePad1){
                    if (toggleGamePad1) {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                        telemetry.addLine("BREAK");
                        toggleGamePad1 = false;
                    } else {
                        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                        telemetry.addLine("FLOAT");
                        toggleGamePad1 = true;
                    }
                releasedGamePad1 = false;
                }
            } else {
                releasedGamePad1 = true;
            }

            LFPower  = Range.clip(speed*(drive + rotate - strafe), -1.0, 1.0) ;
            LBPower  = Range.clip(speed*(drive + rotate + strafe), -1.0, 1.0) ;
            RFPower  = Range.clip(speed*(drive - rotate + strafe), -1.0, 1.0) ;
            RBPower  = Range.clip(speed*(drive - rotate - strafe), -1.0, 1.0) ;

//            if(reverse){
//                int currentIndex = speedList.size() - 1;
//                int maxIndex = 0;
//                double maxValue = -2;
//                for(int i = currentIndex - 200; i <= currentIndex; i++){
//                    if(Math.abs(Double.valueOf(speedList.get(i)[0])) > maxValue){
//                        maxIndex = i;
//                        maxValue = speedList.get(i)[0];
//                    }
//                }
//                LFPower = -0.8 * speedList.get(maxIndex)[0];
//                LBPower = -0.8 * speedList.get(maxIndex)[1];
//                RFPower = -0.8 * speedList.get(maxIndex)[2];
//                RBPower = -0.8 * speedList.get(maxIndex)[3];
//                telemetry.addLine("BREAK APPLIED");
//                LFPower = 0;
//                LBPower = 0;
//                RFPower = 0;
//                RBPower = 0;

//            }

            Double currentSpeed[] = {LFPower, LBPower, RFPower, RBPower};
            speedList.add(currentSpeed);

            LF.setPower(LFPower);
            RF.setPower(RFPower);
            LB.setPower(LBPower);
            RB.setPower(RBPower);

            telemetry.addData("Front Motors", "LF (%.2f), RF (%.2f)", LFPower, RFPower);
            telemetry.addData("Back Motors", "LB (%.2f), RB (%.2f)", LBPower, RBPower);
            telemetry.addData("Controller", "X (%.2f), Y (%.2f)", strafe, drive);
            telemetry.addData("speed:", speed);

            /*if(loop == 200){
                if (reverse){
                    reverse = false;
                }
                telemetry.addData("duration of a loop: %.2f", runtime.milliseconds());
                sleep(100);
                loop = 0;
            }*/
            telemetry.update();
            //loop++;

//            released = true;
        }
    }

    private void decreaseSpeed(double s) {
        double decreased = speed - s;
        if (decreased < 0) {
            speed = 0;
            return;
        }
        speed = decreased;
    }

    private void increaseSpeed(double s) {
        double increased = speed + s;
        if (1 < increased) {
            speed = 1;
            return;
        }
        speed = increased;
    }
}