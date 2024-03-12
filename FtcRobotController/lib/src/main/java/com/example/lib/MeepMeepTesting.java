package com.example.lib;

import static java.lang.Math.min;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);


        double Y_OFFSET = 20;
        double
                stack_x = 50, stack_y = 18 + 30, stack_h = Math.toRadians(-90),
                middlepurple_x = 36, middlepurple_y = 11 + 30, middlepurple_h = Math.toRadians(-90),
                middleyellow_x = 32, middleyellow_y = -88 + 30, middleyellow_h = Math.toRadians(-110),rightpurple_x = 14.5, rightpurple_y = -3, rightpurple_h = Math.toRadians(300);


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 30, 3, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d())//new Pose2d(leftyellow_x-12, leftyellow_y - 1 - Y_OFFSET, Math.toRadians(-55)))
                                .splineToSplineHeading(new Pose2d(rightpurple_x, rightpurple_y, rightpurple_h), rightpurple_h)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}