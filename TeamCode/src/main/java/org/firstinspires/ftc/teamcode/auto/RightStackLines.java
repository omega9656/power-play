/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * Last updated on 1/14/23 Wilcox High School Qualifier #2
 */
@Autonomous(name="Right Stack Lines")
public class RightStackLines extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Robot robot;
    SampleMecanumDrive drive;
    ElapsedTime slidesTime;
    ElapsedTime outTime;

    ElapsedTime intakeTime;
    Pose2d startPose = new Pose2d(34.2, -62.75, Math.toRadians(270));

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    double zoneX = 33;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        slidesTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        outTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(startPose);
        robot = new Robot(hardwareMap);
        robot.init(true, false);

        robot.arm.init();

        zoneX = 34;

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera, 30);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            robot.arm.leftServoProfile.update();
            robot.arm.rightServoProfile.update(robot.arm.leftServoProfile);

            telemetry.addData("left servo curr", robot.arm.leftServo.getPosition());
            telemetry.addData("right servo curr", robot.arm.rightServo.getPosition());
            telemetry.addData("left servo target", robot.arm.leftServoProfile.getTargetPosition());
            telemetry.addData("right servo target", robot.arm.rightServoProfile.getTargetPosition());

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){
            zoneX = 32.8;
        }
        else if(tagOfInterest.id == RIGHT){
            zoneX = 58;
        }
        else if(tagOfInterest.id == LEFT){
            zoneX = 8;
        }
        else {
            zoneX = 34;
        }

        TrajectorySequence t = drive.trajectorySequenceBuilder(startPose).setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.ready();
                    robot.intake.hold();
                })
                // high #1                           18, -7.5
                .splineToLinearHeading(new Pose2d(18, -9, // 19, -7.75
                        //                                               45                    135 -> 150
                        Math.toRadians(Math.toDegrees(startPose.getHeading()))), Math.toRadians(165))
                .setReversed(false)//TODO add .setReversed(false)

                // outtake high #1
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.slides.high();
                    robot.arm.deposit();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> robot.intake.out())
                .waitSeconds(0.5)
                // while moving to cone stack go to intake position
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.stop();
                    robot.arm.intake();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> robot.slides.readyAuto())
                //

                .turn(Math.toRadians(40))

                // cone stack spline #1              61     -14
                .splineToLinearHeading(new Pose2d(58, -14.5, Math.toRadians(startPose.getHeading())), Math.toRadians(0))
                .setReversed(true)//TODO add .setReversed(true)

                // intake #1
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.in();
                    robot.slides.fifthAutoCone();
                })
                // lifts slides after intaking cone
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.hold();
                    robot.slides.readyAuto();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.arm.deposit();
                })
                .waitSeconds(1)

                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    //robot.slides.autoHigh();
                    robot.slides.high();
                    robot.arm.deposit();
                })

                //
                // to low junction
                // high junction #2
                .lineToConstantHeading(new Vector2d(30, -10))

                // outtake high #2

                .lineToLinearHeading(new Pose2d(25, -6.5, Math.toRadians(-63)))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> robot.intake.out())
                .waitSeconds(0.5)
                // turnX -> line to outtake
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.stop();
                    robot.arm.intake();
                })
                .UNSTABLE_addTemporalMarkerOffset(2, () -> robot.slides.readyAuto())
                //


                // cone stack spline #2
                .lineToLinearHeading(new Pose2d(59, -11.25, Math.toRadians(startPose.getHeading()))).setReversed(false)

                // intake #2
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.in();
                    robot.slides.fifthAutoCone();
                })
                // lifts slides after intaking cone
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.hold();
                    robot.slides.readyAuto();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.arm.deposit();
                })
                .waitSeconds(1)

                // high junction #3
                .lineToConstantHeading(new Vector2d(30, -10))

                // outtake high #2

                .lineToLinearHeading(new Pose2d(25, -5, Math.toRadians(-70)))

                // outtake high #3
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    robot.slides.high();
                    robot.arm.deposit();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> robot.intake.out())
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.stop();
                    robot.arm.intake();
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.slides.init())
                .lineToLinearHeading(new Pose2d(33, -15, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(zoneX, -15))
                .build();

        drive.followTrajectorySequenceAsync(t);
        while(opModeIsActive() && !isStopRequested()){
            drive.update();
            robot.arm.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}