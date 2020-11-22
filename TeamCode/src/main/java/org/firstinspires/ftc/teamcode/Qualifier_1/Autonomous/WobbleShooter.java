package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous;

        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

        import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;
        import org.firstinspires.ftc.teamcode.Qualifier_1.Robot;

@Autonomous(name= "WobbleShooter_autonomous")
public class WobbleShooter extends LinearOpMode {
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
        robot.moveAngle(0,-63,0.5);
        robot.moveAngle(20,5,0.5);
        sleep(2000);
        //robot.shootHighGoal(3);
        robot.moveAngle(0,-5,0.5);
        sleep(500);
        stop();
    }



}
