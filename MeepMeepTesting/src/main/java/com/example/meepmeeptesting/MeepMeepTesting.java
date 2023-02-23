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
        Pose2d start = new Pose2d(34.3, -62.016, Math.toRadians(0));
        Pose2d start2 = new Pose2d(34.3, -62, Math.toRadians(270));

        //                           -58
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAnccel, maxAngVel, maxAngAccel, track width
                .setConstraints(48, 30, 55, Math.toRadians(60), 13)
                .followTrajectorySequence(drive -> // start pose
                        drive.trajectorySequenceBuilder(start).setReversed(true)
                                //.splineToSplineHeading(new Pose2d(34.3, -10, Math.toRadians(270)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(34.3, -50, Math.toRadians(270)), Math.toRadians(90))
                                .splineToSplineHeading(new Pose2d(34.3, -10, Math.toRadians(270)), Math.toRadians(90))
                                //.splineToSplineHeading(new Pose2d(28, -12, Math.toRadians(315)), Math.toRadians(90))

                                .setReversed(false)
                                .splineToSplineHeading(new Pose2d(40, -14, Math.toRadians(0)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(58, -14, Math.toRadians(0)), Math.toRadians(0))
                                //.lineToSplineHeading(new Pose2d(34, 0, Math.toRadians(270)))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}