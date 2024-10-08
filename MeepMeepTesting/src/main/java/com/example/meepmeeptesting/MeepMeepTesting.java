package com.example.meepmeeptesting;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.LinkedList;
import java.util.Scanner;

import static com.example.meepmeeptesting.Bots.*;

public class MeepMeepTesting {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f);

        Scanner scanner = new Scanner(System.in);
        int endHeading = 270;
        String input;
        LinkedList<RoadRunnerBotEntity> bots = new LinkedList<>();

        while (true) {
//            System.out.print("enter new pose heading (deg): ");
//            input = scanner.nextLine();
//            poseHeading = (input.isEmpty()) ? poseHeading : Integer.parseInt(input);

            System.out.print("enter new end heading(deg): ");
            input = scanner.nextLine();
            endHeading = (input.isEmpty()) ? endHeading : Integer.parseInt(input);

            if (bots.size() > 0) {
                RoadRunnerBotEntity bot = bots.pop();
                meepMeep.removeEntity(bot);
            }

            RoadRunnerBotEntity newBot = submersibleCycleBot(meepMeep, endHeading);
            bots.add(newBot);
            meepMeep.addEntity(newBot);

            meepMeep.start();


            System.out.println("next cycle");
        }
    }


}