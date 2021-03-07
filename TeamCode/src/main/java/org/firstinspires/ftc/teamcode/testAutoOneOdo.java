package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autotest Only one odo",group = "UltimateGoal")
public class testAutoOneOdo extends testAutoNoOdo{
    AutoOmniMovement autoOmniMovement = new AutoOmniMovement(100, 100, 90);
    Drivetrain drivetrain;
    double time = System.currentTimeMillis();
    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();

        telemetry.addData("what", "Do u want");
        telemetry.update();
        drivetrain = new Drivetrain(robot);

        waitForStart();

        time = System.currentTimeMillis();
        while (opModeIsActive() && time+5000 > System.currentTimeMillis()){
            if (drivetrain.robot.odometryTest) {
                telemetry.addData("Encoder Value", drivetrain.robot.intake.getCurrentPosition());
            }
            telemetry.addData("Encoder Value", drivetrain.robot.intake.getCurrentPosition());
            telemetry.update();
        }
        time = System.currentTimeMillis();
        drivetrain.forward(0);

    }
}
