package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

@Config
@TeleOp(name = "Arm Lift Test Motion Profile", group="pid")
public class ArmLiftTestMotionProfile extends LinearOpMode {


    LiftPIDController liftPIDController;
    ArmPIDController armPIDController;

    public static int armTarget = 0;
    public static int liftTarget = 0;
    public static double maxSpeed = 0.3;
    public static double kp=6, ki=0, kd=0.09;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        liftPIDController = new LiftPIDController(drive.linearSlide, drive.frontSlide, drive.slideOtherer);

        TrapezoidProfile.Constraints m_constraints = new TrapezoidProfile.Constraints(2800, 3000);

        ProfiledPIDController m_controller = new ProfiledPIDController(kp, ki, kd, m_constraints);
m_controller.setTolerance(3);

        drive.tipCenter();
        drive.closeClaw();


        waitForStart();

        drive.closeClaw();
        drive.tipBack();


        while (opModeIsActive() && !isStopRequested()) {
            m_controller.setD(kd);
            m_controller.setI(ki);
            m_controller.setP(kp);

            m_controller.setGoal(armTarget);

           // if(!m_controller.atGoal()) {
                double output = m_controller.calculate(
                        drive.armMotor.getCurrentPosition()  // the measured value
                );
                telemetry.addData("output", output);
                drive.armMotor.setVelocity(output);
                telemetry.addData("arm target", armTarget);
                telemetry.addData("arm position", drive.armMotor.getCurrentPosition());


                telemetry.update();
           // }




            //  telemetry.addData("power ", power);
            telemetry.addData("arm target", armTarget);
            telemetry.addData("arm position", drive.armMotor.getCurrentPosition());


            telemetry.update();

        }


    }

}
