package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.swing.plaf.basic.BasicOptionPaneUI;

public class Meep {
    public static int x = -50, y = 50;
    public static Pose2d
            MiddlePurple = new Pose2d(23, -1, 0),
            MiddleYellow = new Pose2d(14.2, -31.5, Math.toRadians(-60)),

    LeftPurple = new Pose2d(9.5, -7.3, Math.toRadians(-357)),
            LeftYellow = new Pose2d(13.4, -27.5, Math.toRadians(-76)),

    RightPurple = new Pose2d(16.5, 5, Math.toRadians(-314)),
            RightYellow = new Pose2d(20, -31.5, Math.toRadians(-57))
                    ;

    public static Pose2d
            TrussToStack     = new Pose2d(5, 45, -Math.PI/2.f),
            Stack            = new Pose2d(23.5, 75.5, -Math.toRadians(110)),
            Stack2           = new Pose2d(34, 75, -Math.toRadians(110)),
            BackBoardToTruss = new Pose2d(5, 9, -Math.PI/2.f),
            Backdrop         = new Pose2d(12.2, -28, -Math.toRadians(73)),
            TrussToStack_s     = new Pose2d(4, 45, -Math.PI/2.f),
            BackBoardToTruss_s = new Pose2d(4, 9, -Math.PI/2.f)

                    ;

    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, 5, 5, 12.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(Stack2)


                                .build()

                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}