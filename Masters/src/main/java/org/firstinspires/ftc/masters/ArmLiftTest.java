package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

import java.util.Date;

@Config
@Autonomous(name = "Arm Lift Test", group="pid")
public class ArmLiftTest extends LinearOpMode {


    LiftPIDController liftPIDController;
    ArmPIDControllerMotionProfile armPIDController;

    public static int armTarget = 0;
    public static int liftTarget = 0;
    public static double maxSpeed = 0.3;

    public static int maxVel = 25;
    public static int maxAccel=40;
    public static int maxJerk = 100;

    public static double kP=0,kI=0,kD =0;

    public int previousTarget=0;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDControllerMotionProfile(drive.armMotor);
        //drive.tipCenter();
        drive.closeClaw();


        MotionProfile motionProfile=null;
        PIDFController controller =null;

        waitForStart();

        drive.closeClaw();
        drive.tipFront();

        long startTime = new Date().getTime();


        while (opModeIsActive() && !isStopRequested()) {


            armPIDController.setTarget(armTarget);


// in each iteration of the control loop
// measure the position or output variable
// apply the correction to the input variable


            drive.armMotor.setVelocity(armPIDController.calculateVelocity());
//
//            liftPIDController.setTarget(liftTarget);
//
//            double power = liftPIDController.calculatePower();
//
//            drive.linearSlide.setPower(power);
//            drive.frontSlide.setPower(power);
//            drive.slideOtherer.setPower(power);

                //  telemetry.addData("power ", power);
               // telemetry.addData("power", correction);

            liftPIDController.setTarget(liftTarget);
            double power = liftPIDController.calculatePower(drive.linearSlide);
            drive.linearSlide.setPower(power);
            drive.slideOtherer.setPower(liftPIDController.calculatePower(drive.slideOtherer));
            drive.frontSlide.setPower(power);

            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
            telemetry.addData("lift target", liftTarget);
            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            telemetry.update();

        }


    }

}
