package org.firstinspires.ftc.teamcode.UltimateGoalComponents;

import android.widget.ExpandableListView;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.internal.robotBase;
import org.firstinspires.ftc.robotcontroller.internal.RobotComponent;

import java.util.ArrayList;


@SuppressWarnings("MoveFieldAssignmentToInitializer")
public class Drivetrain extends RobotComponent {

   
        public DcMotor frontLeft;
        public DcMotor frontRight;
        public DcMotor backLeft;
        public DcMotor backRight;
        public DcMotor[] motors = new DcMotor[4];

    private double stopBuffer = 0.05;

    static final double COUNTS_PER_INCH = 85.9;

    public Drivetrain(robotBase BASE) {
        super(BASE);
        initMotors();
    }

     void initMotors() {

               frontLeft = base().getMapper().mapMotor("frontLeft");
               motors[0] = frontLeft;

               backLeft = base().getMapper().mapMotor("backLeft");
               motors[1] = backLeft;
         frontRight = base().getMapper().mapMotor("frontRight", DcMotorSimple.Direction.REVERSE);
        motors[2] = frontRight;

         backRight = base().getMapper().mapMotor("backRight", DcMotorSimple.Direction.REVERSE);
        motors[3] = backRight;


        setModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setModes(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setZeroPowerBehaviors(DcMotor.ZeroPowerBehavior.BRAKE);

     }

    public void drive(double forward, double right, double turn) {

        forward = getProcessedInput(forward);
        right = getProcessedInput(right);
        turn = getProcessedInput(turn);

        double leftFrontPower = forward + right + turn;
        double leftBackPower = forward - right + turn;
        double rightFrontPower = forward - right - turn;
        double rightBackPower = forward + right - turn;
        double[] powers = {leftFrontPower, leftBackPower, rightFrontPower, rightBackPower};

        boolean needToScale = false;
        for (double power : powers) {
            if (Math.abs(power) > 1) {
                needToScale = true;
                break;
            }
        }
        if (needToScale) {
            double greatest = 0;
            for (double power : powers) {
                if (Math.abs(power) > greatest) {
                    greatest = Math.abs(power);
                }
            }
            leftFrontPower /= greatest;
            leftBackPower /= greatest;
            rightFrontPower /= greatest;
            rightBackPower /= greatest;
        }
        setPowers(powers);
    }
        public double[] getPowers () {
            double[] powers = {frontLeft.getPower(), backLeft.getPower(), frontRight.getPower(), backRight.getPower()};
            return powers;
        }

        public void setPowers ( double[] powers){
            frontLeft.setPower(powers[0]);
            backLeft.setPower(powers[1]);
            frontRight.setPower(powers[2]);
            backRight.setPower(powers[3]);
        }
        public void setPowers ( double power){
            for (DcMotor m : motors) {
                m.setPower(power);
            }
        }

        public double getAverageEncoders (ArrayList < DcMotor > dcmotors) {
            double sum = 0;
            for (DcMotor m : dcmotors) {
                sum += Math.abs(m.getCurrentPosition());
            }
            return (sum / (double) (dcmotors.size()));
        }


        public void stop () {
            setPowers(0);
        }

        public void setModes (DcMotor.RunMode runMode){
            for (DcMotor motor : motors) {
                motor.setMode(runMode);
            }
        }
        public void setZeroPowerBehaviors (DcMotor.ZeroPowerBehavior behavior){
            for (DcMotor motor : motors) {
                motor.setZeroPowerBehavior(behavior);
            }
        }

        private double getProcessedInput ( double hardInput){
            hardInput = Range.clip(hardInput, -1, 1);
            hardInput = Math.pow(hardInput, 3);
            return hardInput;
        }
}
