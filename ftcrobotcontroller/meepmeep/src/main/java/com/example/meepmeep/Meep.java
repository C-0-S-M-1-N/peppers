package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.swing.plaf.basic.BasicOptionPaneUI;

public class Meep {
    public static double x = 14, y = 62, rot = -90;

    public static Pose2d
            MiddlePurple = new Pose2d(24, -1, 0),
            MiddleYellow = new Pose2d(38.5, -33, Math.toRadians(234)),

    RightPurple = new Pose2d(44.7, -5, Math.toRadians(190)),
            RightYellow = new Pose2d(37, -32, Math.toRadians(233)),

    LeftPurple = new Pose2d(12, 8, Math.toRadians(33)),
            LeftYellow = new Pose2d(42, -30, Math.toRadians(245)),

    BackDropGate = new Pose2d(51, 5, Math.toRadians(270)),
            StackGate = new Pose2d(51, 43, Math.toRadians(270)),
            Stack = new Pose2d(51, 76.5, Math.toRadians(270)),
            Stack2 = new Pose2d(41, 74.5, Math.toRadians(291)),
            Backdrop = new Pose2d(47, -33, Math.toRadians(230));

    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, 5, 5, 12.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Backdrop)
                                .setReversed(true)


//                                .splineToSplineHeading(BackDropGate, Math.toRadians(90))
                                .lineToSplineHeading(BackDropGate)
                                .splineToSplineHeading(StackGate, Math.toRadians(90))
                                .splineToSplineHeading(Stack, Math.toRadians(90))
                                .build()


                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}