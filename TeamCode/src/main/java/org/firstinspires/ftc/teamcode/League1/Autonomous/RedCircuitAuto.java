package org.firstinspires.ftc.teamcode.League1.Autonomous;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Common.Point;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Subsystems.MecDrive;

import java.io.FileNotFoundException;

@Autonomous(name = "Red Right Auto")
public class RedCircuitAuto extends OpModeWrapper {
    Robot robot;
    MecDrive drive;

    @Override
    protected void onInitialize() throws FileNotFoundException {
        robot = new Robot(hardwareMap);
        drive = new MecDrive(hardwareMap, robot, false, telemetry);

        robot.start();

    }

    @Override
    protected void onStart() {
        drive.rotate(Math.PI/2, 0.2);
        drive.newMoveToPosition(new Point(0, -1000), 0.5);
        //drive.setPower(-0.2, -0.2, -0.2, -0.2);

       /* while(opModeIsActive()){
            LynxModule.BulkData data = robot.getBulkPacket(true);
            telemetry.addData("main fl ", data.getMotorCurrentPosition(0));
            telemetry.addData("main fr ", data.getMotorCurrentPosition(1));
            telemetry.addData("main bl ", data.getMotorCurrentPosition(2));
            telemetry.addData("main br ", data.getMotorCurrentPosition(3));
            telemetry.update();

        }
*/


        /*
        drive.rotate(Math.PI/4, 0.2);
        sleep(1000);
        drive.rotate(Math.PI/2, 0.2);
        drive.newMoveToPosition(new Point(-1000, 0),0.5);
        drive.newMoveToPosition(new Point(0, -1000), 0.5);

         */

        //drive.newMoveToPosition(new Point(-1000, -1000), 0.5);

    }

    @Override
    protected void onStop() {
        drive.setPower(0, 0, 0, 0);
        drive.writeLoggerToFile();

    }
}
