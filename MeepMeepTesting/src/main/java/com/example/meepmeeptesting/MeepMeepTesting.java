package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.HeadingInterpolator;
import com.acmerobotics.roadrunner.path.heading.LinearInterpolator;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        //coneToHighSpline(meepMeep);
        startToHigh(meepMeep);
    }

    public static void startToHigh(MeepMeep meepMeep){
        Pose2d start = new Pose2d(34, -62, Math.toRadians(270));

        //                           -58
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAnccel, maxAngVel, maxAngAccel, track width
                .setConstraints(48, 30, 55, Math.toRadians(60), 13)
                .followTrajectorySequence(drive -> // start pose
                        drive.trajectorySequenceBuilder(start).setReversed(true)
                                // start position to drop off indicated by barcode
                                // move to intake cone #2
                                .splineToLinearHeading(new Pose2d(start.getX()-6, start.getY()+56,
                                        //                                               45                    135
                                        Math.toRadians(Math.toDegrees(start.getHeading()))), Math.toRadians(135))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static void coneToHighSpline(MeepMeep meepMeep){
        Pose2d start = new Pose2d(-60, -12, Math.toRadians(180));

        //                           -58
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAnccel, maxAngVel, maxAngAccel, track width
                .setConstraints(48, 30, 55, Math.toRadians(60), 13)
                .followTrajectorySequence(drive -> // start pose
                        drive.trajectorySequenceBuilder(start).setReversed(true)
                                // start position to drop off indicated by barcode
                                // move to intake cone #2
                                .splineToSplineHeading(new Pose2d(start.getX()+32, start.getY()+5.5,
                                        Math.toRadians(Math.toDegrees(start.getHeading()+45))), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}