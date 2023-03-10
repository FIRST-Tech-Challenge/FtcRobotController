package org.firstinspires.ftc.teamcode.subsystems.swerve;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.*;

public class SwerveModule {
    private final DcMotorEx top;
    private final DcMotorEx bottom;

    private final RobotConfig r;
    private final RobotConstants.moduleSides side;

    public final Log log;

    private final Angle absoluteAngle;
    private final Angle continuousAngle;
    /**
     * includes the adjustment for the robot's heading
     */
    private final Angle adjustedTargetAngle;
    private double moduleHeadingError;

    private double power;

    private boolean moduleFlipped;
    private boolean braked;
    private final Angle brakeAngle;

    private double previousDistance;
    private double currentDistance;


    public SwerveModule(RobotConfig r, RobotConstants.moduleSides side){
        this.r = r;
        this.side = side;

        if (side == RobotConstants.moduleSides.LEFT) {
            this.log = new Log(RobotConstants.configuredSystems.LEFT_MODULE, "Absolute Heading", "Flipping Heading", "Robot Heading Adjusted Target Angle", "Target Heading Adjusted Target Angle", "Error", "Top Power", "Bottom Power");
            top = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.leftTop);
            bottom = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.leftBottom);
        }
        else {
            this.log = new Log(RobotConstants.configuredSystems.RIGHT_MODULE,"Absolute Heading", "Flipping Heading", "Robot Heading Adjusted Target Angle", "Target Heading Adjusted Target Angle", "Error", "Top Power", "Bottom Power");
            top = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.rightTop);
            bottom = r.opMode.hardwareMap.get(DcMotorEx.class, ConfigNames.rightBottom);
        }

        top.setDirection(DcMotorEx.Direction.REVERSE);
        //bottom.setDirection(DcMotorEx.Direction.REVERSE);

        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        moduleFlipped = false;
        moduleHeadingError = 0;
        absoluteAngle = new Angle(Angle.angleType.ABSOLUTE);
        continuousAngle = new Angle(Angle.angleType.CONTINUOUS);
        brakeAngle = new Angle(Angle.angleType.ABSOLUTE);
        adjustedTargetAngle = new Angle(0, Angle.angleType.ABSOLUTE);
        previousDistance = 0;
        currentTime = System.nanoTime();
    }


    public void readEncoders(){
        double topPos = top.getCurrentPosition();
        double bottomPos = bottom.getCurrentPosition();

        if(side == RobotConstants.moduleSides.LEFT) {
            absoluteAngle.update(90 + ((topPos - bottomPos) / RobotConstants.ticksPerDegreeLeftSwerve));
            continuousAngle.update((topPos - bottomPos) / RobotConstants.ticksPerDegreeLeftSwerve);
        }
        else{
            absoluteAngle.update(90 + ((topPos - bottomPos) / RobotConstants.ticksPerDegreeRightSwerve));
            continuousAngle.update((topPos - bottomPos) / RobotConstants.ticksPerDegreeRightSwerve);
        }

        if(side == RobotConstants.moduleSides.LEFT) {
            currentDistance = (topPos + bottomPos)* RobotConstants.ticksPerMMLeftSwerve;
        }
        else{
            currentDistance = (topPos + bottomPos)* RobotConstants.ticksPerMMRightSwerve;
        }
        log.logData(0, absoluteAngle.value);
    }

    public void resetEncoders(){
        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getInstanceDistance(){
        double instanceDistance = currentDistance-previousDistance;
        previousDistance = currentDistance;
        return instanceDistance;
    }

    public Angle getCurrentModuleAngle(){
        if(!moduleFlipped){
            return absoluteAngle;
        }
        else{
            return new Angle(absoluteAngle.value + 180, Angle.angleType.ABSOLUTE);
        }
    }

    public void manualDrive(double planarY, double planarX, double turnPower, double robotHeadingError, double throttle, boolean holdPos){
        double powerSum;

        gamepadAnalysis(planarY, planarX, holdPos);

        if(RobotConstants.robotFlipping){
            turnPower *= (robotHeadingError/90);
        }
        else{
            turnPower *= (robotHeadingError/180);
        }

        powerSum = Math.abs(power) + Math.abs(turnPower);

        if(powerSum>1){
            power /= powerSum;
            turnPower /= powerSum;
        }
        pDrive(power, turnPower, throttle, robotHeadingError);
    }

    private void gamepadAnalysis(double planarY, double planarX, boolean holdPos){
        Angle targetAngle = Angle.atanHandler(planarX, planarY);
        power = Math.sqrt((planarX*planarX)+(planarY * planarY));

        if(holdPos){
            top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (planarY == 0.0 && planarX == 0.0 && !braked) {
                brakeAngle.update(absoluteAngle.value);
                braked = true;
            }
            if (!(targetAngle.value == brakeAngle.value)) {
                braked = false;
            }
            if(braked){
                targetAngle.update(brakeAngle.value);
            }
        }
        else{
            top.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            bottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        // TODO: fix robot position tracking
        if(SwerveDriveBase.enabledModules == RobotConstants.enabledModules.BOTH){
            adjustedTargetAngle.update((targetAngle.value)- RobotConfig.robotPose2D.angle.value+90);
        }
        else{
            adjustedTargetAngle.update(targetAngle.value);
        }
    }

    private void pDrive(double power, double turnPower, double throttle, double robotHeadingError){
        double driveSign;
        log.logData(2, adjustedTargetAngle.value);


        if(SwerveDriveBase.enabledModules == RobotConstants.enabledModules.BOTH){
            adjustedTargetAngle.update(adjustedTargetAngle.value + ((turnPower+ RobotConstants.minimumHeadingEmphasis)/(1+ RobotConstants.minimumHeadingEmphasis))*robotHeadingError);
        }


        log.logData(3, adjustedTargetAngle.value);

        if (Math.abs(moduleHeadingError)>90){
            moduleFlipped = !moduleFlipped;
        }
        if (!moduleFlipped) {
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new Angle(absoluteAngle.value, Angle.angleType.ABSOLUTE));
            if(side == RobotConstants.moduleSides.LEFT){
                driveSign = 1;
            }
            else {
                driveSign = -1;
            }
            log.logData(1, absoluteAngle.value);
        }
        else{
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new Angle(absoluteAngle.value +180, Angle.angleType.ABSOLUTE));
            if(side == RobotConstants.moduleSides.LEFT){
                driveSign = -1;
            }
            else {
                driveSign = 1;
            }
            log.logData(1, new Angle(absoluteAngle.value+180, Angle.angleType.ABSOLUTE).value);
        }
        log.logData(4, moduleHeadingError);


        if(side == RobotConstants.moduleSides.LEFT){
            robotHeadingError *= -1;
        }

        double headingPower = RobotConstants.robotHeading_kP * ((turnPower + RobotConstants.minimumHeadingApproachPower) / (1 + RobotConstants.minimumHeadingApproachPower)) * (robotHeadingError);
        double moduleAnglePower = (RobotConstants.moduleAngle_kP * ((power + RobotConstants.minimumModuleAngleApproachPower) / (1 + RobotConstants.minimumModuleAngleApproachPower)) * (moduleHeadingError));

        if(side == RobotConstants.moduleSides.LEFT){
            headingPower *= -1;
        }
        else {
            headingPower *= 1;
        }

        double topPower = ((power * driveSign) + moduleAnglePower + headingPower);
        double bottomPower = ((power * driveSign) - moduleAnglePower + headingPower);

        double max = Math.max(Math.abs(topPower), Math.abs(bottomPower));
        if(max >1){
            bottomPower /= max;
            topPower /= max;
        }

        log.logData(5, topPower);
        log.logData(6, bottomPower);

        top.setPower(topPower * throttle);
        bottom.setPower(bottomPower * throttle);
        log.updateLoop(true);
    }

    private double currentVelocity;
    private double currentTime;

    public void ffDrive(double turnPower, double robotHeadingError, double xPositionError, double yPositionError, double targetVelocity, double maxVelocity){
        if(! (SwerveDriveBase.enabledModules == RobotConstants.enabledModules.BOTH)){
            return;
        }
        Angle targetAngle = Angle.atanHandler(xPositionError, yPositionError);
        adjustedTargetAngle.update((targetAngle.value) - RobotConfig.robotPose2D.angle.value + 90);

        double previousTime = currentTime;
        currentTime = System.nanoTime();
        double timeDifference = currentTime- previousTime;
        double distanceTravelled = RobotConfig.robotPose2D.getDifference(RobotConfig.previousRobotPose2D);
        double previousVelocity = currentVelocity;

        currentVelocity = (distanceTravelled/timeDifference);
        double currentAcceleration = ((currentVelocity-previousVelocity) / timeDifference);

        double driveSign;

        double powerSum;

        if(RobotConstants.robotFlipping){
            turnPower *= (robotHeadingError/90);
        }
        else{
            turnPower *= (robotHeadingError/180);
        }

        log.logData(2, adjustedTargetAngle.value);

        if (Math.abs(moduleHeadingError)>90){
            moduleFlipped = !moduleFlipped;
        }
        if (!moduleFlipped) {
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new Angle(absoluteAngle.value, Angle.angleType.ABSOLUTE));
            driveSign = 1;
            log.logData(1, absoluteAngle.value);
        }
        else{
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new Angle(absoluteAngle.value +180, Angle.angleType.ABSOLUTE));
            driveSign = -1;
            log.logData(1, new Angle(absoluteAngle.value+180, Angle.angleType.ABSOLUTE).value);
        }
        log.logData(4, moduleHeadingError);


        if(side == RobotConstants.moduleSides.LEFT){
            robotHeadingError *= -1;
        }

        double headingPower = RobotConstants.robotHeading_kP * ((power + RobotConstants.minimumHeadingApproachPower) / (1 + RobotConstants.minimumHeadingApproachPower)) * (robotHeadingError);
        double moduleAnglePower = (RobotConstants.moduleAngle_kP * ((power + RobotConstants.minimumModuleAngleApproachPower) / (1 + RobotConstants.minimumModuleAngleApproachPower)) * (moduleHeadingError));

        double outputVelocity;
        double outputAcceleration;

        if(targetVelocity > RobotConstants.maxSwerveVelocity){
            targetVelocity = RobotConstants.maxSwerveVelocity;
        }

        if(targetVelocity > currentVelocity){
            outputVelocity = currentVelocity + RobotConstants.maxSwerveAcceleration*timeDifference;
            outputAcceleration = RobotConstants.maxSwerveAcceleration;
        }
        else if(targetVelocity < currentVelocity && Math.sqrt(xPositionError*xPositionError + yPositionError*yPositionError) <= (currentVelocity * currentVelocity) / (2 * RobotConstants.maxSwerveAcceleration)){
            outputVelocity = currentVelocity - RobotConstants.maxSwerveAcceleration*timeDifference;
            outputAcceleration = RobotConstants.maxSwerveAcceleration;
        }
        else{
            outputVelocity = RobotConstants.maxSwerveVelocity;
            outputAcceleration = 0;
        }

        power = (outputVelocity + outputAcceleration)/(RobotConstants.maxSwerveVelocity);

        powerSum = Math.abs(power) + Math.abs(turnPower);

        if(powerSum>1){
            power /= powerSum;
            turnPower /= powerSum;
        }

        adjustedTargetAngle.update(adjustedTargetAngle.value +((turnPower+ RobotConstants.minimumHeadingEmphasis)/(1+ RobotConstants.minimumHeadingEmphasis))* RobotConfig.robotPose2D.angle.value);

        log.logData(3, adjustedTargetAngle.value);

        double topPower = ((power * driveSign) + moduleAnglePower + headingPower);
        double bottomPower = ((power * driveSign) - moduleAnglePower + headingPower);

        double max = Math.max(Math.abs(topPower), Math.abs(bottomPower));
        if(max >1){
            bottomPower /= max;
            topPower /= max;
        }

        log.logData(5, topPower);
        log.logData(6, bottomPower);

        top.setPower(topPower);
        bottom.setPower(bottomPower);
        log.updateLoop(true);
    }
}
