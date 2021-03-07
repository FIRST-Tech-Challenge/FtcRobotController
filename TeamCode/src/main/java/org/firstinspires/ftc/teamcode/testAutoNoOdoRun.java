package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autotestpowerthing",group = "UltimateGoal")
public class testAutoNoOdoRun extends testAutoNoOdo{
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
        autoOmniMovement.addTurnPower(90);
        drivetrain.setMotorPowers(autoOmniMovement.wheelPowers[3], autoOmniMovement.wheelPowers[1], autoOmniMovement.wheelPowers[2], autoOmniMovement.wheelPowers[0]);
        while (opModeIsActive() && time+5000 > System.currentTimeMillis()){
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[0]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[1]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[2]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[3]);
            if (drivetrain.robot.odometryTest) {
                telemetry.addData("Encoder Value", drivetrain.robot.horizontal.getCurrentPosition());
            }
            telemetry.update();
        }
        autoOmniMovement.resetPowers();
        time = System.currentTimeMillis();
        autoOmniMovement.addTurnPower(-90);
        drivetrain.setMotorPowers(autoOmniMovement.wheelPowers[3], autoOmniMovement.wheelPowers[1], autoOmniMovement.wheelPowers[2], autoOmniMovement.wheelPowers[0]);
        while (opModeIsActive() && time+5000 > System.currentTimeMillis()){
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[0]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[1]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[2]);
            telemetry.addData("Powers", autoOmniMovement.wheelPowers[3]);
            telemetry.update();
        }
        drivetrain.forward(0);

    }
}
