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
    ArmPIDController armPIDController;

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
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();


        MotionProfile motionProfile=null;
        PIDFController controller =null;

        waitForStart();

        drive.closeClaw();
        drive.tipBack();

        long startTime = new Date().getTime();


        while (opModeIsActive() && !isStopRequested()) {

            if (armTarget != previousTarget) {
                previousTarget = armTarget;


                motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(drive.armMotor.getCurrentPosition(), 0, 0),
                        new MotionState(armTarget, 0, 0),
                        25,
                        40,
                        100);
                startTime = new Date().getTime();
            }
            PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
// create the controller
            controller = new PIDFController(coeffs);

// specify the setpoint
            controller.setTargetPosition(armTarget);


// in each iteration of the control loop
// measure the position or output variable
// apply the correction to the input variable

            if (motionProfile!=null && controller!=null) {
                MotionState state = motionProfile.get((new Date().getTime() - startTime) / 1000);

                controller.setTargetPosition(state.getX());
                controller.setTargetVelocity(state.getV());
                controller.setTargetAcceleration(state.getA());



                telemetry.addData("x", state.getX());
                telemetry.addData("v", state.getV());
                telemetry.addData("a", state.getA());


                double correction = controller.update(drive.armMotor.getCurrentPosition());

//            armPIDController.setTarget(armTarget);
//            double armPower = armPIDController.calculateVelocity();
//            if (armTarget<0){
//                if (armPower<0){
//                    armPower= Math.max(armPower, -maxSpeed);
//                }
//                else armPower = Math.min(armPower, maxSpeed);
//            }
            drive.armMotor.setPower(correction);
//
//            liftPIDController.setTarget(liftTarget);
//
//            double power = liftPIDController.calculatePower();
//
//            drive.linearSlide.setPower(power);
//            drive.frontSlide.setPower(power);
//            drive.slideOtherer.setPower(power);

                //  telemetry.addData("power ", power);
                telemetry.addData("power", correction);
            }
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());

            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            telemetry.update();

        }


    }

}
