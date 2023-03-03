package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.teleop.OmegaTeleopModular;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(name = "Good Mid Auto")
public class RightMidAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    Robot robot;
    SampleMecanumDrive drive;

    Pose2d startPose = new Pose2d(33.851016, -61.73, Math.toRadians(270));
    //Pose2d startPose = new Pose2d(34.3, -62.016, Math.toRadians(0));

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

    double parkZone = 34.2;

    PathState pathState;
    ElapsedTime timer;

    AprilTagDetection tagOfInterest = null;
    TrajectorySequence park;

    enum PathState {
        START_TO_HIGH,
        CONE_MID_TO_CONE_STACK,
        HIGH_TO_CONE_STACK,
        FIND_CONE,
        CONE_STACK_TO_MID,
        PARK;
    }

    int conesDeposited = 0;
    // 6 = 1+5 auto
    public static int MAX_CONES_DEPOSITED = 6;
    boolean gotHeading = false;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        drive = new SampleMecanumDrive(hardwareMap);
        robot = new Robot(hardwareMap);
        robot.init(true, false);

        robot.arm.init();

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

        double MID_X = 29;
        double MID_Y = -19.75;

        double CONE_STACK_Y = -12.35;

        TrajectorySequence startToHigh = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> robot.intake.hold())

                // continious path to high junction
                .splineToSplineHeading(new Pose2d(33.674016, -20, Math.toRadians(270)), Math.toRadians(90))
                // x = 29.25
                .splineToSplineHeading(new Pose2d(26.7, -8.65, Math.toRadians(315)), Math.toRadians(135))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.slides.autoHigh();
                    robot.arm.autoDeposit();
                })

                // outtake + deposit
                .UNSTABLE_addTemporalMarkerOffset(0.03, () -> {
                    robot.intake.out();
                })

                .build();

        TrajectorySequence startHighToConeStack = drive.trajectorySequenceBuilder(startToHigh.end())
                .setReversed(false)

                //.waitSeconds(0.15)

                // lower
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })
                .waitSeconds(0.1)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.lowAndIntake();
                })

                // continuous path to cone stack
                //.splineToSplineHeading(new Pose2d(32.4, -12.4, Math.toRadians(315)), Math.toRadians(-45))
                .splineToSplineHeading(new Pose2d(29.7, -11.65, Math.toRadians(315)), Math.toRadians(-45))
                //.splineToSplineHeading(new Pose2d(40, -12.4, Math.toRadians(340)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))

                // pick up cone                         -0.7
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    robot.arm.intake();
                    robot.intake.in();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intake.hold();
                    robot.slides.autoHigh();
                })

                .waitSeconds(0.2)
                .build();

        TrajectorySequence coneStacktoMid = drive.trajectorySequenceBuilder(startHighToConeStack.end())
                // TODO debug reversed
                .setReversed(true)

                // continuous path to high junction same position as end of startToHigh
                // TODO, added line below
                .splineToSplineHeading(new Pose2d(48, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(180))

                // good below                          .5     -8.00
                .splineToSplineHeading(new Pose2d(MID_X, MID_Y, Math.toRadians(45)), Math.toRadians(225))

                // deposit
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    robot.slides.autoMid();
                    robot.arm.autoDeposit();
                })

                // outtake + deposit
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.out();
                })

                .build();

        TrajectorySequence midToConeStack = drive.trajectorySequenceBuilder(coneStacktoMid.end())
                .setReversed(false)

                // lower
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.slides.lowAndIntake();
                })

                // continuous path to cone stack
                //.splineToSplineHeading(new Pose2d(MID_X-.5, MID_Y+.5, Math.toRadians(45)), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(44, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))

                // pick up cone                         -0.775
                .UNSTABLE_addTemporalMarkerOffset(-0.65, () -> {
                    robot.arm.intake();
                    robot.intake.in();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intake.hold();
                    robot.slides.autoHigh();
                })

                .waitSeconds(0.2)

                .build();

        TrajectorySequence lastMidToConeStack = drive.trajectorySequenceBuilder(coneStacktoMid.end())
                .setReversed(false)

                // lower
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.slides.lowAndIntake();
                })

                // continuous path to cone stack
                //.splineToSplineHeading(new Pose2d(MID_X-.2, MID_Y+.2, Math.toRadians(45)), Math.toRadians(45))
                //.splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                //.splineToLinearHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))

                // pick up cone                         -0.7
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    robot.arm.intake();
                    robot.intake.in();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intake.hold();
                    robot.slides.autoHigh();
                })

                .waitSeconds(0.2)

                .build();

        TrajectorySequence parkMid = drive.trajectorySequenceBuilder(coneStacktoMid.end())
                .setReversed(false)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.slides.lowAndIntake();
                })

                // continuous path to cone stack
                .splineToSplineHeading(new Pose2d(34, -15.75, Math.toRadians(270)), Math.toRadians(45))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> robot.arm.intake())
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(coneStacktoMid.end())
                .setReversed(false)

                // lower
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.slides.lowAndIntake();
                })

//                // continuous path to cone stack
//                .splineToSplineHeading(new Pose2d(28.7, -15.75, Math.toRadians(45)), Math.toRadians(45))
//                .splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(44, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(57.85, CONE_STACK_Y, Math.toRadians(0)), Math.toRadians(0))

                // pick up cone                         -0.7
                .UNSTABLE_addTemporalMarkerOffset(-1.3, () -> {
                    robot.intake.in();
                })

                .build();



        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(coneStacktoMid.end())
                .setReversed(false)

                // lower
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.arm.init();
                    robot.intake.stop();
                })

                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.slides.lowAndIntake();
                })

                // continuous path to cone stack
                .splineToLinearHeading(new Pose2d(8, -11, Math.toRadians(0)), Math.toRadians(180))

                // pick up cone                         -0.7
                .UNSTABLE_addTemporalMarkerOffset(-1.3, () -> {
                    robot.intake.in();
                })

                .build();

        if(tagOfInterest == null || tagOfInterest.id == MIDDLE){
            park = parkMid;
        }
        else if(tagOfInterest.id == RIGHT){
            park = parkRight;
        }
        else if(tagOfInterest.id == LEFT){
            park = parkLeft;
        }

        pathState = PathState.START_TO_HIGH;

        // ADDED Pose estimate right before start
        drive.setPoseEstimate(startPose);
        drive.followTrajectorySequenceAsync(startToHigh);

        while(opModeIsActive() && !isStopRequested()){
            Pose2d poseEstimate = drive.getPoseEstimate();

            switch (pathState) {
                case START_TO_HIGH:
                    // arm deposit and slides lift in trajectory sequence
                    if (!drive.isBusy()) {
                        // outtake for 0.25 seconds (in trajectory sequence)
                        conesDeposited++;
                        drive.followTrajectorySequenceAsync(startHighToConeStack);
                        pathState = PathState.CONE_STACK_TO_MID;
                    }
                    break;
                case CONE_MID_TO_CONE_STACK:
                    if (!drive.isBusy()) {
                        conesDeposited++;
                        // on last cone path
                        if (conesDeposited < 4) {
                            drive.followTrajectorySequenceAsync(midToConeStack);
                            pathState = PathState.CONE_STACK_TO_MID;
                        } else if (conesDeposited < 6) {
                            drive.followTrajectorySequenceAsync(lastMidToConeStack);
                            pathState = PathState.CONE_STACK_TO_MID;
                        } else {
                            drive.followTrajectorySequenceAsync(park);
                            pathState = PathState.PARK;
                        }
                    }
                    break;
                case CONE_STACK_TO_MID:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(coneStacktoMid);
                        pathState = PathState.CONE_MID_TO_CONE_STACK;
                    }
                    break;
                case PARK:
                    if(!drive.isBusy() && !gotHeading){
                        OmegaTeleopModular.startHeading = Math.toDegrees(poseEstimate.getHeading());
                        gotHeading = true;
                    }
                    break;
            }

            drive.update();
            robot.arm.update();
            robot.slides.setPowerProportional();


            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("slides left curr", robot.slides.leftSlides.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("slides right curr", robot.slides.rightSlides.getCurrent(CurrentUnit.AMPS));
            telemetry.update();

            //OmegaTeleopModular.startHeading = Math.toDegrees(poseEstimate.getHeading());
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
