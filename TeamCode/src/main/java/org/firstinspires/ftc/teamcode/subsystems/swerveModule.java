package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamUtil.*;

public class swerveModule {
    private final DcMotorEx top;
    private final DcMotorEx bottom;

    private final robotConfig r;
    private final robotConstants.moduleSides side;

    public final log log;

    private final angle absoluteAngle;
    public final angle continuousAngle;
    /**
     * includes the adjustment for the robot's heading
     */
    private final angle adjustedTargetAngle;
    private double moduleHeadingError;

    private double power;

    private boolean moduleFlipped;
    private boolean braked;
    private final angle brakeAngle;

    private double previousDistance;
    private double currentDistance;


    public swerveModule(robotConfig r, robotConstants.moduleSides side){
        this.r = r;
        this.side = side;

        if (side == robotConstants.moduleSides.LEFT) {
            this.log = new log(robotConstants.configuredSystems.LEFT_MODULE, "Absolute Heading", "Flipping Heading", "Robot Heading Adjusted Target Angle", "Target Heading Adjusted Target Angle", "Error", "Top Power", "Bottom Power");
            top = r.hardwareMap.get(DcMotorEx.class, configNames.leftTop);
            bottom = r.hardwareMap.get(DcMotorEx.class, configNames.leftBottom);
        }
        else {
            this.log = new log(robotConstants.configuredSystems.RIGHT_MODULE,"Absolute Heading", "Flipping Heading", "Robot Heading Adjusted Target Angle", "Target Heading Adjusted Target Angle", "Error", "Top Power", "Bottom Power");
            top = r.hardwareMap.get(DcMotorEx.class, configNames.rightTop);
            bottom = r.hardwareMap.get(DcMotorEx.class, configNames.rightBottom);
        }

        top.setDirection(DcMotorEx.Direction.REVERSE);
        bottom.setDirection(DcMotorEx.Direction.REVERSE);

        top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        top.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottom.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        moduleFlipped = false;
        moduleHeadingError = 0;
        absoluteAngle = new angle(angle.angleType.ABSOLUTE);
        continuousAngle = new angle(angle.angleType.CONTINUOUS);
        brakeAngle = new angle(angle.angleType.ABSOLUTE);
        adjustedTargetAngle = new angle(0, angle.angleType.ABSOLUTE);
        previousDistance = 0;
        currentTime = System.nanoTime();
    }


    public void readEncoders(){
        double topPos = top.getCurrentPosition();
        double bottomPos = bottom.getCurrentPosition();

        if(side == robotConstants.moduleSides.LEFT) {
            absoluteAngle.update(90 + ((topPos - bottomPos) / robotConstants.ticksPerDegreeLeftSwerve));
            continuousAngle.update((topPos - bottomPos) / robotConstants.ticksPerDegreeLeftSwerve);
        }
        else{
            absoluteAngle.update(90 + ((topPos - bottomPos) / robotConstants.ticksPerDegreeRightSwerve));
            continuousAngle.update((topPos - bottomPos) / robotConstants.ticksPerDegreeRightSwerve);
        }

        if(side == robotConstants.moduleSides.LEFT) {
            currentDistance = (topPos + bottomPos)*robotConstants.ticksPerMMLeftSwerve;
        }
        else{
            currentDistance = (topPos + bottomPos)*robotConstants.ticksPerMMRightSwerve;
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

    public angle getCurrentModuleAngle(){
        if(!moduleFlipped){
            return absoluteAngle;
        }
        else{
            return new angle(absoluteAngle.value + 180, angle.angleType.ABSOLUTE);
        }
    }

    public void manualDrive(double planarY, double planarX, double turnPower, double robotHeadingError, double throttle, boolean holdPos){
        double powerSum;

        gamepadAnalysis(-planarY, planarX, holdPos);

        if(robotConstants.robotFlipping){
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
        angle targetAngle = angle.atanHandler(planarX, -planarY);
        power = Math.sqrt((planarX*planarX)+(planarY*planarY));

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
        if(swerveDriveBase.enabledModules == robotConstants.enabledModules.BOTH){
            adjustedTargetAngle.update((targetAngle.value)- robotConfig.robotPose2D.angle.value+90);
        }
        else{
            adjustedTargetAngle.update(targetAngle.value);
        }

        adjustedTargetAngle.update(targetAngle.value);

    }

    private void pDrive(double power, double turnPower, double throttle, double robotHeadingError){
        double driveSign;
        log.logData(2, adjustedTargetAngle.value);


        if(swerveDriveBase.enabledModules == robotConstants.enabledModules.BOTH){
            adjustedTargetAngle.update(adjustedTargetAngle.value + ((turnPower+robotConstants.minimumHeadingEmphasis)/(1+robotConstants.minimumHeadingEmphasis))*robotHeadingError);
        }


        log.logData(3, adjustedTargetAngle.value);

        if (Math.abs(moduleHeadingError)>90){
            moduleFlipped = !moduleFlipped;
        }
        if (!moduleFlipped) {
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new angle(absoluteAngle.value, angle.angleType.ABSOLUTE));
            if(side == robotConstants.moduleSides.LEFT){
                driveSign = 1;
            }
            else {
                driveSign = -1;
            }
            log.logData(1, absoluteAngle.value);
        }
        else{
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new angle(absoluteAngle.value +180, angle.angleType.ABSOLUTE));
            if(side == robotConstants.moduleSides.LEFT){
                driveSign = -1;
            }
            else {
                driveSign = 1;
            }
            log.logData(1, new angle(absoluteAngle.value+180, angle.angleType.ABSOLUTE).value);
        }
        log.logData(4, moduleHeadingError);


        if(side == robotConstants.moduleSides.LEFT){
            robotHeadingError *= -1;
        }

        double headingPower = robotConstants.robotHeading_kP * ((turnPower + robotConstants.minimumHeadingApproachPower) / (1 + robotConstants.minimumHeadingApproachPower)) * (robotHeadingError);
        double moduleAnglePower = (robotConstants.moduleAngle_kP * ((power + robotConstants.minimumModuleAngleApproachPower) / (1 + robotConstants.minimumModuleAngleApproachPower)) * (moduleHeadingError));

        if(side == robotConstants.moduleSides.LEFT){
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
        if(! (swerveDriveBase.enabledModules == robotConstants.enabledModules.BOTH)){
            return;
        }
        angle targetAngle = angle.atanHandler(xPositionError, yPositionError);
        adjustedTargetAngle.update((targetAngle.value) - robotConfig.robotPose2D.angle.value + 90);

        double previousTime = currentTime;
        currentTime = System.nanoTime();
        double timeDifference = currentTime- previousTime;
        double distanceTravelled = robotConfig.robotPose2D.getDifference(robotConfig.previousRobotPose2D);
        double previousVelocity = currentVelocity;

        currentVelocity = (distanceTravelled/timeDifference);
        double currentAcceleration = ((currentVelocity-previousVelocity) / timeDifference);

        double driveSign;

        double powerSum;

        if(robotConstants.robotFlipping){
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
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new angle(absoluteAngle.value, angle.angleType.ABSOLUTE));
            driveSign = 1;
            log.logData(1, absoluteAngle.value);
        }
        else{
            moduleHeadingError = adjustedTargetAngle.angleShortDifference(new angle(absoluteAngle.value +180, angle.angleType.ABSOLUTE));
            driveSign = -1;
            log.logData(1, new angle(absoluteAngle.value+180, angle.angleType.ABSOLUTE).value);
        }
        log.logData(4, moduleHeadingError);


        if(side == robotConstants.moduleSides.LEFT){
            robotHeadingError *= -1;
        }

        double headingPower = robotConstants.robotHeading_kP * ((power + robotConstants.minimumHeadingApproachPower) / (1 + robotConstants.minimumHeadingApproachPower)) * (robotHeadingError);
        double moduleAnglePower = (robotConstants.moduleAngle_kP * ((power + robotConstants.minimumModuleAngleApproachPower) / (1 + robotConstants.minimumModuleAngleApproachPower)) * (moduleHeadingError));

        double outputVelocity;
        double outputAcceleration;

        if(targetVelocity > robotConstants.maxSwerveVelocity){
            targetVelocity = robotConstants.maxSwerveVelocity;
        }

        if(targetVelocity > currentVelocity){
            outputVelocity = currentVelocity + robotConstants.maxSwerveAcceleration*timeDifference;
            outputAcceleration = robotConstants.maxSwerveAcceleration;
        }
        else if(targetVelocity < currentVelocity && Math.sqrt(xPositionError*xPositionError + yPositionError*yPositionError) <= (currentVelocity * currentVelocity) / (2 * robotConstants.maxSwerveAcceleration)){
            outputVelocity = currentVelocity - robotConstants.maxSwerveAcceleration*timeDifference;
            outputAcceleration = robotConstants.maxSwerveAcceleration;
        }
        else{
            outputVelocity = robotConstants.maxSwerveVelocity;
            outputAcceleration = 0;
        }

        power = (outputVelocity + outputAcceleration)/(robotConstants.maxSwerveVelocity);

        powerSum = Math.abs(power) + Math.abs(turnPower);

        if(powerSum>1){
            power /= powerSum;
            turnPower /= powerSum;
        }

        adjustedTargetAngle.update(adjustedTargetAngle.value +((turnPower+robotConstants.minimumHeadingEmphasis)/(1+robotConstants.minimumHeadingEmphasis))* robotConfig.robotPose2D.angle.value);

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
