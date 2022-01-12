package org.firstinspires.ftc.teamcode.auto.paths;

import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logData;
import static org.firstinspires.ftc.teamcode.Configs.newConfig.NewAutoDriveUtils.logLine;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Configs.newConfig.BaseNewOpMode;
import org.firstinspires.ftc.teamcode.Configs.newConfig.HardwareNew;

@Autonomous(name= "Right Position Both Alliances")
public class RightPositionBothAlliances extends BaseNewOpMode {

    private final HardwareNew robot = new HardwareNew(true);
    private final ElapsedTime runtime = new ElapsedTime();


    /**
     * {@inheritDoc}
     * @author karthikperi
     */
    @SuppressLint("DefaultLocale")

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            doAutoDriving(this,0, 'R');
        }



    }

    public void doAutoDriving(BaseNewOpMode OpMode,int level, char direction) {
        moveToShippingElement(OpMode, direction);
        dropToShippingElement(level);
    }

    private void dropToShippingElement(int level) {
        if (level == 1) {
            // drop at the bottom most
            // claw.release()
        } else if (level == 2) {
            // drop at the middle one.
        } else if (level == 3) {
            // drop at the top one.
        } else {
        }
    }


    private void moveToShippingElement(BaseNewOpMode OpMode, char direction) {
        if (direction == 'L') {
            encoderDrive(this, 0.5, 3, 3, 5);
            robot.turnRight(OpMode,45, 0.1);
            encoderDrive(this, 0.5, 21, 21, 5);
        } else if (direction == 'R'){
            encoderDrive(this, 0.5, 3, 3, 5);
            robot.turnLeft(OpMode,45, 0.1);
            encoderDrive(this, 0.5, 21, 21, 5);
        }



        }
    public HardwareNew getRobot() {
        return robot;
    }
    }


