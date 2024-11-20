package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto Pink/Aqua")
public class AutoPinkAqua extends AutoCommandOpMode {
    @Override
    public void logic() {
        schedule(new SequentialCommandGroup(
                /*step 1: go to central position with RedBlue*/
                drive(12)
                , turnCCW(90)
                , drive(9)
                , turnCW(90)

                /*step 2: go to cage (this has a different name)*/
                , drive(16)
                /* do something with the arm*/

                /*step 3: go to first sample*/
                , drive(-9)
                , turnCCW(90)
                , drive(-33)
                , turnCCW(45)
                , drive(-11)
                , turnCW(135)

                /*step 4: drive first sample to box*/
                , drive(-32)

                /*step 5: go to second sample*/
                , drive(48)
                , turnCCW(45)
                , drive(-10)
                , turnCW(45)

                /*step 6: drive second sample to box*/
                , drive(-40)

                /*
                *step 7: go to third sample*
                , drive(45)
                , turnCCW(45)
                , drive(-10)
                , turnCW(45)

                *step 8: drive third sample to box*
                , drive(-40)


                *step 9: park*
                */
        ));
    }
}