package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.TangentInterpolator;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAnccel, maxAngVel, maxAngAccel, track width
                .setConstraints(48, 30, 55, Math.toRadians(60), 13)
                .followTrajectorySequence(drive -> // start pose
                                drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))
                                        // start position to drop off indicated by barcode
                                        .back(25) // 48
                                        // turn and drop off cone #1
                                        .strafeLeft(25)
                                        // move to intake cone #2
                                        .lineToLinearHeading(new Pose2d(-50, -11, Math.toRadians(180)))
                                        //.lineToConstantHeading(new Vector2d(-60, -12.5))
                                        // move to outtake cone #2
                                        .lineToConstantHeading(new Vector2d(-35, -12.5))
                                        // turn and outtake cone #2
                                        .turn(Math.toRadians(60))
                                        // turn back to intake cone #3
                                        .turn(Math.toRadians(-60))
                                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_KAI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}