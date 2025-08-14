package org.firstinspires.ftc.teamcode.subsystems.mecanum;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.odometry.PinPointOdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.pidcore.PIDCore;

import static org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

//Here are the subsystems used
public class MecanumCommand {

    // create a class to consolidate subsystems
    private MecanumSubsystem mecanumSubsystem;
    private PinPointOdometrySubsystem pinPointOdo;

    // hardware is owned by test and pass down to subsystems
    private Hardware hw;

    private LinearOpMode opMode;
    private ElapsedTime elapsedTime;
    public double xFinal;
    public double yFinal;
    private double thetaFinal;
    private double velocity;

    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;

   // subsystem class would be pass into the functions
    public MecanumCommand(LinearOpMode opmode, Hardware hw) {
        this.hw = hw;
        this.mecanumSubsystem = new MecanumSubsystem(hw);
        this.pinPointOdo = new PinPointOdometrySubsystem(hw);
        this.opMode = opmode;
        elapsedTime = new ElapsedTime();
        xFinal = pinPointOdo.getX();
        yFinal = pinPointOdo.getY();
        thetaFinal = pinPointOdo.getHeading();
        velocity = 0;
        mecanumSubsystem.turnOffInternalPID();
    }

    //Command used to update constants
    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta){
        MecanumConstants.kpx = kpx;
        MecanumConstants.kdx = kdx;
        MecanumConstants.kix = kix;
        MecanumConstants.kpy = kpy;
        MecanumConstants.kdy = kdy;
        MecanumConstants.kiy = kiy;
        MecanumConstants.kptheta = kptheta;
        MecanumConstants.kdtheta = kdtheta;
        MecanumConstants.kitheta = kitheta;
        mecanumSubsystem.updatePIDConstants();
    }

    public void turnOffInternalPID(){
        mecanumSubsystem.turnOffInternalPID();
    }

    //uses imu for heading
    public void pidPinPointProcessIMU(double imuAngle){

        ex = mecanumSubsystem.globalXController.outputPositional(xFinal, pinPointOdo.getX());
        ey =  mecanumSubsystem.globalYController.outputPositional(yFinal, pinPointOdo.getY());
        etheta = mecanumSubsystem.globalThetaController.outputPositional(thetaFinal, pinPointOdo.getHeading());

        if ((isXReached())) {
            globalXController.integralReset();
        }
        if ((isYReached())) {
            globalYController.integralReset();
        }

        if (isThetaReached()) {
            globalThetaController.integralReset();
        }

        if (isXPassed()) {
            globalXController.activateIntegral();
        } else {
            globalXController.deactivateIntegral();
        }

        if (isYPassed()) {
            globalYController.activateIntegral();
        } else {
            globalYController.deactivateIntegral();
        }

        if (isThetaPassed()) {
            globalThetaController.activateIntegral();
        } else {
            globalThetaController.deactivateIntegral();
        }

        double max = Math.max(Math.abs(ex), Math.abs(ey));
        if (max > velocity) {
            double scalar = velocity / max;
            ex *= scalar;
            ey *= scalar;
            etheta *= scalar;
        }
        moveGlobalPartialPinPoint(true, ex, ey, etheta);

    }


    // AUTO COMMANDS
}
