package org.firstinspires.ftc.teamcode;// https://first-tech-challenge.github.io/SkyStone/  This is the link to ALL metered of FTC

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="FTC 14133 2021 PID Tuning", group="Auto")
@Disabled
public class FTC_14133_2021_PID_Tuning extends LinearOpMode {
    private DcMotorEx lb = null;        // Sets the variables of the mecanum wheels
    private DcMotorEx rb = null;
    private DcMotorEx lf = null;
    private DcMotorEx rf = null;
    static final double MOTOR_TICK_COUNT = 2800;        //
    private DcMotorEx shooter = null;         // Sets the variable of the shooter


    void ForwardorBackwards(double distance, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving forward/backwards
        //  double distance= 5; //(in)
        double encodercounts = distance * 60.3686819388;//(1/(75*(1/25.4)))*560;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(encodercountsint);
        lf.setPower(speed);        //Sets the power for the left front wheel
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the right front wheel
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the left back wheel
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the right back wheel
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || lb.isBusy()) {

            }

    }




    void Rotate(double turn, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving left/right
        //NOT DONE
        double encodercounts = turn * 13.18; // test iteratively
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(-encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (lf.isBusy() || lb.isBusy()) {
            //run until motors arrive at position
        }
    }

    void Strafing(double Strafe, double speed) {
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Driving left/right
        //Positive is Strafing left negative is Strafing right
        double encodercounts = Strafe * 60.3686819388 * 1.4142135623730950488016887242097;
        int encodercountsint = (int) encodercounts;
        lf.setTargetPosition(-encodercountsint);
        lf.setPower(speed);        //
        rf.setTargetPosition(encodercountsint);
        rf.setPower(speed);        //Sets the power for the Long arm
        lb.setTargetPosition(encodercountsint);
        lb.setPower(speed);        //Sets the power for the Long arm
        rb.setTargetPosition(-encodercountsint);
        rb.setPower(speed);        //Sets the power for the Long arm
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (lf.isBusy() || lb.isBusy()) {
            //run until motors arrive at position
        }


    }



    public void waitForStart() {
    }

    public void runOpMode() {
        lf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lf");       //sets the names of the motors on the hardware map
        rf = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rf");
        lb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "lb");
        rb = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "rb");
        shooter = (DcMotorEx)hardwareMap.get(DcMotorEx.class, "shooter");

        final double driveP = 16;        //PID values will change, these are filler values
        final double driveI = 0;
        final double driveD = 5;
        final double driveF = 0;
        PIDFCoefficients drivePIDF = new PIDFCoefficients(driveP, driveI, driveD, driveF);
        lf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        rf.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        lb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);
        rb.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, drivePIDF);

        final double ShooterP = 10;        //PID values will change, these are filler values
        final double ShooterI = 0;
        final double ShooterD = 0;
        final double ShooterF = 0;
        PIDFCoefficients ShooterPIDF = new PIDFCoefficients(ShooterP, ShooterI, ShooterD, ShooterF);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, ShooterPIDF);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter.setDirection(DcMotorEx.Direction.REVERSE);            //sets the directions of the motors
        lf.setDirection(DcMotorEx.Direction.FORWARD);
        rf.setDirection(DcMotorEx.Direction.REVERSE);
        lb.setDirection(DcMotorEx.Direction.FORWARD);
        rb.setDirection(DcMotorEx.Direction.REVERSE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);        //Since this is the first time using the encoder we start it up
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        shooter.setVelocity(2200);
        sleep(2000);
        shooter.setVelocity(1500);
        sleep(2000);
        shooter.setVelocity(2200);
        sleep(2000);
        shooter.setVelocity(1500);
        sleep(2000);
        shooter.setVelocity(2200);
        sleep(2000);
        shooter.setVelocity(1500);
        sleep(2000);

        shooter.setVelocity(0);



        sleep(4000);



        ForwardorBackwards(25, 1);

        sleep(1000);

        ForwardorBackwards(-25, 1);

        sleep(1000);

        ForwardorBackwards(25, 1);

        sleep(1000);

        ForwardorBackwards(-25, 1);

        sleep(1000);

        ForwardorBackwards(25, 1);

        sleep(1000);

        ForwardorBackwards(-25, 1);

        sleep(1000);

        ForwardorBackwards(25, 1);

        sleep(1000);

        ForwardorBackwards(-25, 1);



        sleep(4000);



        Strafing(10, 1);

        sleep(1000);

        Strafing(-10, 1);

        sleep(1000);

        Strafing(10, 1);

        sleep(1000);

        Strafing(-10, 1);

    }
}
