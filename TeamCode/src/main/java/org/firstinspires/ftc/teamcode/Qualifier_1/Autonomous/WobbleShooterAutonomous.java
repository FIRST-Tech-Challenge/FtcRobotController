package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
        import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name= "WobbleShooter_autonomous")
public class WobbleShooterAutonomous extends LinearOpMode {
    final boolean debug = true;

    @Override
    public void runOpMode(){

        Robot robot=new Robot(this);
        Odometry odom = new Odometry();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        odom.init(this);
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();
        waitForStart();
        robot.moveAngleOdometry(0,-52,0.5);
        robot.moveAngleOdometry(20,7,0.5);
        //robot.shootHighGoal(3);
        robot.moveAngleOdometry(0,-7,0.5);
        sleep(500);
        stop();
    }



}
