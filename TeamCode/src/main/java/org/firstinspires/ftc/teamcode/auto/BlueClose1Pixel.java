package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.V4B;

@Autonomous(name="Blue Close 1Pixel")
public class BlueClose1Pixel extends CommandOpMode {


    //Add Motors and servos not for drivebase here
    SampleMecanumDrive drive;
    private IMU imu;
    GamepadEx gamepad;

    private DcMotor fl, fr, bl, br;
    int tagId = 0; //Dw about it rn
    Slides s = new Slides(hardwareMap);
    Outtake o = new Outtake(gamepad, hardwareMap);

    V4B v4b = new V4B(hardwareMap);

    @Override
    public void initialize() {
        drive = new SampleMecanumDrive(hardwareMap);
        //motors and servos hardwareMap stuff here
        fl= hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "bl");
//        Impasta autoImp = new Impasta(fl, fr, bl, br, imu);


        while (!isStarted() && !isStopRequested()) {
            //dw about it
        }

        UnworkingTrajectories.generateTrajectories(drive); //Loads trajectories from trajectories file

        TrajectorySequence prop;
        TrajectorySequence score;
        TrajectorySequence park;

        switch(tagId) {
            case 0:
                prop = UnworkingTrajectories.propCloseLeftBlue;
                score = UnworkingTrajectories.scoreCloseLeftBlue;
                park = UnworkingTrajectories.parkCloseLeftBlue;
                break;
            case 1:
                prop = UnworkingTrajectories.propCloseMidBlue;
                score = UnworkingTrajectories.scoreCloseMidBlue;
                park = UnworkingTrajectories.parkCloseMidBlue;
                break;
            case 2:
                prop = UnworkingTrajectories.propCloseRightBlue;
                score = UnworkingTrajectories.scoreCloseRightBlue;
                park = UnworkingTrajectories.parkCloseRightBlue;
                break;
            default:
                prop = UnworkingTrajectories.propCloseLeftBlue;
                score = UnworkingTrajectories.scoreCloseLeftBlue;
                park = UnworkingTrajectories.parkCloseLeftBlue;
        }


        schedule(new SequentialCommandGroup ( //Makes the following code run one after another, like norma
                new TrajectorySequenceCommand(drive, prop),
                new TrajectorySequenceCommand(drive, score),
                new InstantCommand(() -> {
                    s.goToPosition(Slides.SlidePos.LOW);
//                    v4b.togglePosition();
                    o.open();
                    new WaitCommand(5000);
                    o.close();
//                    v4b.togglePosition();
                    s.switchMaxMin();

                }),
                new TrajectorySequenceCommand(drive, park)
                //add ur code here, look back at last year if u need help
            )
        );
    }
}