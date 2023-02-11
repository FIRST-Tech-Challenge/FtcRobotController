package org.firstinspires.ftc.masters;

import static org.firstinspires.ftc.masters.BadgerConstants.ARM_BACK;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

@Config
@Autonomous(name = "Arm Lift Test")
public class ArmLiftTest extends LinearOpMode {


    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    public static int armTarget = 0;
    public static int liftTarget = 0;
    public static double maxSpeed = 0.3;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);
        armPIDController = new ArmPIDController(drive.armMotor);
        drive.tipCenter();
        drive.closeClaw();


        waitForStart();

        drive.closeClaw();
        drive.tipBack();


        while (opModeIsActive() && !isStopRequested()) {


            armPIDController.setTarget(armTarget);
            double armPower = armPIDController.calculateVelocity();
            if (armTarget<0){
                if (armPower<0){
                    armPower= Math.max(armPower, -maxSpeed);
                }
                else armPower = Math.min(armPower, maxSpeed);
            }
            drive.armMotor.setPower(armPower);

            liftPIDController.setTarget(liftTarget);

            double power = liftPIDController.calculatePower();

            drive.linearSlide.setPower(power);
            drive.frontSlide.setPower(power);
            drive.slideOtherer.setPower(power);

            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());
            telemetry.addData("lift target", liftTarget);
            telemetry.addData(" lift position", drive.linearSlide.getCurrentPosition());

            telemetry.update();

        }


    }

}
