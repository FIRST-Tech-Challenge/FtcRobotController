package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.OdoPod;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name ="RightSideAuto")
@Config
public class OfficialAutoTwo extends PowerPlay_AprilTagDetectionDeposit {

    private Arm armControl;
    private Slide slideControl;
    private Claw clawControl;

    public static double xFirstLinear = 63;
    public static double yFirstLinear =-1.9;
    public static double angle = 203;
    public static double xSecondToJunction = 53;
    public static double ySecondToJunction = 4;
    public static double xThirdToJunction = 54;
    public static double yThirdToJunction =  4;
    public static double xFourthToJunction = 54;
    public static double yFourthToJunction =  4;
    public static double xParkZoneOne=48.5;
    public static double yParkZoneOne=26;
    public static double xParkZoneThree=-46;
    public static double yParkZoneThree=-25;
    public static double angleConeStack = 91.45;
    public static double xSecondPark = 54;
    public static double ySecondPark = 0;
    public static int arm = 925;
    public static double angleSecond = 180;
    public static double angleThird = 180;
    public static int strafeThird = -28;
    public static double cycleThreeAngle = 213;


    public void initialize(){
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawControl = new Claw(hardwareMap);
        OdoPod odoControl = new OdoPod(hardwareMap);
    }

    public void scan(){
        super.runOpMode();
    }

    @Override
    public void runOpMode()
    {
        Pose2d startPose = new Pose2d(0,0,Math.toRadians(180));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        //Trajectories
        initialize();

        TrajectorySequence junction = bot.trajectorySequenceBuilder(startPose)
                //If 13.53 V, y = -3.5
                // Mechanisms deposit preload at high junction with a 0.65 delay
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    slideControl.setCustom(2400);
                    armControl.setExtake();
                    clawControl.toggleWristRotate();

                })
                // MOVES TO HIGH JUNCTION FOR FIRST TIME
                .lineToLinearHeading(new Pose2d(xFirstLinear, yFirstLinear, Math.toRadians(270)))
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1370); // Overrides current target position of slides
                })
                .waitSeconds(.35)
                //opens claw
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, ()->{
                    armControl.setIntake();
                    clawControl.toggleWristRotate();
                })
                .waitSeconds(0.15)
                // MOVES TO CONE STACK
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{
                    slideControl.setCustom(740);
                    armControl.setIntake();
                })
                .waitSeconds(0.25)
                .strafeRight(14.5)
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })






                // GOING TO HIGH JUNCTION FOR THE SECOND TIME
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1370);
                })
                .waitSeconds(.45)
                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{ //0.7 old value
                    slideControl.setCustom(2400); // 2nd time at high junction
                    armControl.setCustom(arm);
                    clawControl.toggleWristRotate();
                })

                .lineToLinearHeading(new Pose2d(xSecondToJunction,ySecondToJunction,Math.toRadians(angle)))
                .UNSTABLE_addTemporalMarkerOffset(0.15,()->{
                    slideControl.setCustom(1370);
                })
                .waitSeconds(0.35) // OLD VALUE: 0.25
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })

                .waitSeconds(0.25) // OLD VALUE: 0.15
                // PREPPING PIDS FOR CONE STACK \\
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{ // OLD VALUE: 0.25
                    clawControl.toggleWristRotate();
                    slideControl.setCustom(630);// OLD VALUE: 620
                    armControl.setIntake();
                })
                .waitSeconds(0.35) // OLD VALUE: 0.25
                // GOING TO THE CONE STACK \\
                .lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(268.55)))
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1310); // RAISES SLIDES BEFORE GOING TO THE JUNCTION FOR THE THIRD TIME
                })



                //GOING TO JUNCTION FOR THIRD TIME
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0.3,()->{
                    slideControl.setCustom(2400);
                    armControl.setCustom(arm);
                    clawControl.toggleWristRotate();
                })
                .lineToLinearHeading(new Pose2d(xThirdToJunction,yThirdToJunction,Math.toRadians(angle)))

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1370);
                })
                .waitSeconds(.35)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })

                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.25,()->{

                    clawControl.toggleWristRotate();
                    slideControl.setCustom(490);
                    armControl.setIntake();
                })
                .waitSeconds(0.25)

                .lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(268.55)))
                .forward(28)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1310);
                })
                .waitSeconds(.25)
                .UNSTABLE_addTemporalMarkerOffset(0.7,()->{
                    slideControl.setCustom(2400);
                    armControl.setCustom(arm);
                    clawControl.toggleWristRotate();
                })
                .waitSeconds(0.5)

                .lineToLinearHeading(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(cycleThreeAngle)))

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setCustom(1370);
                })
                .waitSeconds(.35)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    clawControl.toggleOpenClose();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0,()->{

                    clawControl.toggleWristRotate();

                })
                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    slideControl.setIntakeOrGround();
                    armControl.setIntake();
                })
                .waitSeconds(0.25)



                .addTemporalMarker(() -> {
                    if(tagUse == 1){
                        TrajectorySequence zoneOne = bot.trajectorySequenceBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
                                .lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(180)))
                                .forward(28)
                                .build();
                        bot.followTrajectorySequenceAsync(zoneOne);
                    }else if(tagUse == 2) {
                        TrajectorySequence zoneTwo = bot.trajectorySequenceBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
                                .lineToLinearHeading(new Pose2d(48.5 ,0, Math.toRadians(angleSecond)))
                                .build();
                        bot.followTrajectorySequenceAsync(zoneTwo);
                    }else{
                        Trajectory zoneThree = bot.trajectoryBuilder(new Pose2d(xFourthToJunction,yFourthToJunction,Math.toRadians(angle)))
                                .lineToLinearHeading(new Pose2d(48.5 ,-25, Math.toRadians(268.55)))
                                .build();
                        bot.followTrajectoryAsync(zoneThree);
                    }
                })
                .waitSeconds(3)

                .build();

        scan();

        waitForStart();
        bot.followTrajectorySequenceAsync(junction);
        //bot.followTrajectorySequenceAsync(goToConeStack);
        //bot.followTrajectorySequence(conesToJunction);

        while(opModeIsActive()  && !isStopRequested()){
            bot.update();
            armControl.update(telemetry);
            telemetry.addLine("run");
            telemetry.update();
            slideControl.update(telemetry);
        }

    }
}