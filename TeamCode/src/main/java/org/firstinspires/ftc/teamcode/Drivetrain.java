package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

public class Drivetrain {
    HardwareMapV2 robot;
    public Drivetrain(HardwareMapV2 robot) { this.robot = robot; }
    public enum slapperPos{
        OUT, IN
    }
    public enum tiltDirect{
        UP, DOWN
    }
    public enum moveDirection {
        FORWARD, BACKWARD, LEFT, RIGHT
    }
    static int pos = 0;
    static final double     P_TURN_COEFF            = 0.03;
    static final double     HEADING_THRESHOLD       = 1 ;

    public void forward(double power){
        setMotorPowers(power, power, power, power);
    }
    public void stop(){
        forward(0);
    }
    public void setMotorPowers(double lb, double rb, double lf, double rf){
        robot.frontRight.setPower(rf);
        robot.backRight.setPower(rb);
        robot.frontLeft.setPower(lf);
        robot.backLeft.setPower(lb);
    }
    public void moveDirect(moveDirection direction, double power){
        switch (direction) {
            case FORWARD:
                setMotorPowers(power, power, power, power);
                break;
            case BACKWARD:
                setMotorPowers(-power, -power, -power, -power);
                break;
            case LEFT:
                setMotorPowers(power, -power, -power, power);
                break;
            case RIGHT:
                setMotorPowers(-power, power, power, -power);
                break;
        }
    }
    public void moveSlapper(slapperPos Slapper){
        switch (Slapper){
            case IN:
                robot.slapper.setPosition(0.35);
                break;
            case OUT:
                robot.slapper.setPosition(0.0);
                break;
        }
    }
    public void spin(boolean right, double power){
        power = (right) ? power : -power;
        setMotorPowers(power, -power, power, -power);
    }
    public void outtakeAll(double power){
        outtakeAll(power, power);
    }
    public void outtakeAll(double conveyor, double outtake){
        robot.outtake.setPower(outtake);
    }
    public void newOuttakeAll(double power, int cycles){
        robot.outtake.setPower(power);
        pause(300);
        for (int i=0; i<cycles; i++){
            singleCycle();
        }
        robot.outtake.setPower(0.0);
    }
    public void singleCycle(){
        moveSlapper(slapperPos.OUT);
        pause(200);
        moveSlapper(slapperPos.IN);
        pause(100);
    }
    public void tilt(double pos){
        if (0.0 <= pos && pos <= 4.0) {
            robot.leftTilt.setPosition(pos);
            robot.rightTilt.setPosition(pos);
        }
    }
    public void tiltpos(tiltDirect direct){
        if (direct == tiltDirect.UP && pos != 2){
           pos++;
        }else if(direct == tiltDirect.DOWN && pos != 0){
            pos--;
        }
        tilt((pos==0) ? 0.3 : (pos==1) ? 0.35 : 0.4);
    }
    public void incrementtilt(double amount){
        incrementtilt(amount, amount);
    }

    public void incrementtilt(double amountRT, double amountLT){
        amountLT = (robot.leftTilt.getPosition()+amountLT<0 || robot.leftTilt.getPosition()+amountLT>1.0) ? 0 : amountLT;
        amountRT = (robot.rightTilt.getPosition()+amountRT<0 || robot.rightTilt.getPosition()+amountRT>1.0) ? 0 : amountRT;
        robot.leftTilt.setPosition(robot.leftTilt.getPosition()+amountLT);
        robot.rightTilt.setPosition(robot.rightTilt.getPosition()+amountRT);
    }

    public void incrementaltiltRight(double amount){
        robot.rightTilt.setPosition(robot.rightTilt.getPosition()+amount);
    }
    public void incrementaltiltLeft(double amount){
        robot.leftTilt.setPosition(robot.leftTilt.getPosition()+amount);
    }

    public void pause(double milis){
        double time = System.currentTimeMillis() + milis;
        while (time >= System.currentTimeMillis()) {}
    }
    // NOT USED; gets average gyro value for more accurate angles
    public double getAverageGyro(){
        /*int sum = robot.realgyro.getIntegratedZValue() + robot.realgyro2.getIntegratedZValue();
        return sum/2;*/
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, DEGREES);
        double heading = angles.firstAngle;
        return heading;
    }

    // checks distance for gyroTurn/Drive - slows down when closer to end
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
        }

        // Send desired speeds to motors.
        robot.frontLeft.setPower(leftSpeed);
        robot.frontRight.setPower(rightSpeed);
        robot.backLeft.setPower(leftSpeed);
        robot.backRight.setPower(rightSpeed);


//        // Display it for the driver.
//        telemetry.addData("Target", "%5.2f", angle);
//        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
//        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed);

        return onTarget;
    }

    //show the error in gyro angle
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAverageGyro();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */

    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
