package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import teamcode.common.PurePursuit.MovementVars;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.1;

    /*
    This has most of the relevant information regarding a 4 wheel Mechanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotor fl, fr, bl, br;

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

    }

    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr,bl,br};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3 * Math.PI / 4;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
    }

    /*
    this exists because of the differences between the FTC controller and raw vectors
     */
    public void setPowerPurePursuit(Vector2D velocity, double turnValue){
        turnValue = turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3 * Math.PI / 4;

        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),- (power * cos + turnValue),
                -(power * cos - turnValue), (power * sin + turnValue));
    }



    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(brPow);
    }




    /**
     *
     * @param angle angle the robot should TURN TO in radians
     * @param power
     */
    public void rotate(double angle, double power){
        while(!isNear(Localizer.thisLocalizer().getGlobalRads(), angle)){
            setPower(power, -power, power, -power);
        }
        setPower(0,0,0,0);
    }

    private boolean isNear(double globalRads, double angle) {
        return Math.abs(globalRads - angle) < ANGULAR_TOLERANCE;
    }
}
