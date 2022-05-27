package org.firstinspires.ftc.team417_2021;

public class Robot {

    double curAngle = 0;
    double currentHypotenuse = 0;
    double currentX = 0;
    double currentY = 0;
    private double initialHeading;

    MasterOpMode master;

    // instantiated masterOpMode to resolve static issues because we are making a new robot object
    public Robot(MasterOpMode masterOpMode){
        master = masterOpMode;
    }
    public void setInitialAngle() {
        initialHeading = master.imu.getAngularOrientation().firstAngle;

    }
    public double getInitialHeading() {
        return initialHeading;
    }

    // correct our initial angle depending on where we are starting by the field
    public double getCorrectedHeading(){
        return master.imu.getAngularOrientation().firstAngle - initialHeading;
    }
    public void setCorrectedHeading(double initialHeading) {
        this.initialHeading = initialHeading;
    }

    public void updatePosition() {
        // divide by 4 because we are averaging them (scaling)
        curAngle = getCorrectedHeading();
        currentHypotenuse = ((float) master.motorFL.getCurrentPosition() + master.motorBL.getCurrentPosition() + master.motorFR.getCurrentPosition() + master.motorBR.getCurrentPosition() ) / ( 4 * master.COUNTS_PER_INCH );
        currentY = Math.cos(Math.toRadians(curAngle)) * currentHypotenuse;
        currentX = Math.sin(Math.toRadians(curAngle)) * currentHypotenuse;
    }
}
