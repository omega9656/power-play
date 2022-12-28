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
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotOld;
import org.firstinspires.ftc.teamcode.hardware.ServoProfiler;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Right Stack")
public class RightConeStack extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    RobotOld robot;
    SampleMecanumDrive drive;
    ElapsedTime slidesTime;
    ElapsedTime outTime;

    ElapsedTime intakeTime;
    //Pose2d startPose = new Pose2d(-35, -62, Math.toRadians(270));
    //Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
    Pose2d startPose = new Pose2d(-35, 60, Math.toRadians(270));

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

    AprilTagDetection tagOfInterest = null;

    Pose2d poseEstimate;

    enum State {
        WALL_TO_HIGH,
        TURN_TO_DROP_CONE,
        DROP_CONE,
        BACK_UP_FROM_HIGH,
        TURN_TO_CONE_STACK,
        TO_CONE_STACK,
        INTAKE,
        LIFT_INTAKE,
        BACK_FROM_INTAKE,
        TURN_TO_HIGH_FROM_INTAKE,
        PARK,
        IDLE
    }

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
        robot = new RobotOld();
        robot.init(hardwareMap, true, false);

        robot.leftS = new ServoProfiler(robot.leftServo);
        robot.leftS.setConstraints(1, 1, 1.25);
        robot.rightS = new ServoProfiler(robot.rightServo);
        robot.rightS.setConstraints(1, 1, 1.25);

        robot.setServoPos(0);

        robot.leftS.setTargetPosition(0.4);
        robot.rightS.setTargetPosition(0.4);

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
            robot.leftS.update();
            robot.rightS.update(robot.leftS);

            telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
            telemetry.addData("right servo curr", robot.rightS.getCurrentPosition());
            telemetry.addData("left servo target", robot.leftS.getTargetPosition());
            telemetry.addData("right servo target", robot.rightS.getTargetPosition());

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

        // move servos back down low
        robot.leftS.setConstraints(1, 1, .75);
        robot.rightS.setConstraints(1, 1, .75);

        // first trajectory, moves to high from wall to drop cone
        Trajectory toDeposit = drive.trajectoryBuilder(startPose)
                .back(49)
                .build();

        Trajectory dropCone = drive.trajectoryBuilder(toDeposit.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .back(14) // 9.75
                .build();

        Trajectory dropCone2 = drive.trajectoryBuilder(toDeposit.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .back(11)
                .build();

        Trajectory backCone = drive.trajectoryBuilder(dropCone.end())
                .forward(15) // 10.5
                .build();

        Trajectory toConeStack = drive.trajectoryBuilder(backCone.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .forward(25.5)
                .build();

        Trajectory backConeStack = drive.trajectoryBuilder(toConeStack.end())
                .back(25.5)
                .build();

        Trajectory leftZone = drive.trajectoryBuilder(backCone.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .back(25)
                .build();

        Trajectory rightZone = drive.trajectoryBuilder(backCone.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .forward(30)
                .build();

        Trajectory middleZone = drive.trajectoryBuilder(backCone.end().plus(new Pose2d(0, 0, Math.toRadians(45))))
                .forward(1.5)
                .build();



        int cones = 0;

        // just added
        robot.leftSlides.setPower(.9);
        robot.rightSlides.setPower(.9);
        robot.leftSlides.setTargetPosition(280);
        robot.rightSlides.setTargetPosition(280);
        // here
        robot.intake.setPower(-0.2);

        State currentState = State.WALL_TO_HIGH; // trajectory 1
        drive.followTrajectoryAsync(toDeposit);
        while(opModeIsActive() && !isStopRequested()){

            switch(currentState){
                case WALL_TO_HIGH:
                    // checks if drive is completed finishing moving to
                    // cone and then moves
                    if(!drive.isBusy()){
                        currentState = State.TURN_TO_DROP_CONE;
                        //drive.followTrajectoryAsync(dropCone);
                        drive.turnAsync(Math.toRadians(45));
//                        slidesTime.reset();
//                        outTime.reset();
                        robot.leftSlides.setTargetPosition(1670);
                        robot.rightSlides.setTargetPosition(1670);
                        robot.leftS.setTargetPosition(0.85);
                        robot.rightS.setTargetPosition(0.85);
                    }
                    break;
                case TURN_TO_DROP_CONE:
                    if(!drive.isBusy()){
                        slidesTime.reset();
                        outTime.reset();
                        if(cones == 0) {
                            drive.followTrajectoryAsync(dropCone);
                        }
                        else {
                            drive.followTrajectoryAsync(dropCone2);
                        }
                        currentState = State.DROP_CONE;
                    }
                    break;
                case DROP_CONE:
//                    // waits for turn to stop and slides+v4b to get in position
//                    // TODO if v4b gets to outtake before slides get to pos, add v4b timer
                    if(!drive.isBusy() && slidesTime.milliseconds() > 1000){
                        robot.intake.setPower(0.6);
                        if(cones == 0 && outTime.milliseconds() > 2500){
                            robot.intake.setPower(0);
                            currentState = State.BACK_UP_FROM_HIGH;
                            robot.leftS.setTargetPosition(0);
                            robot.rightS.setTargetPosition(0);
                            cones++;
                        }
                        else if(cones == 1 && outTime.milliseconds() > 4000){
                            robot.intake.setPower(0);
                            currentState = State.BACK_UP_FROM_HIGH;
                            robot.leftS.setTargetPosition(0);
                            robot.rightS.setTargetPosition(0);
                            cones++;
                        }
                    }
                    break;
                case BACK_UP_FROM_HIGH:
                    if(!drive.isBusy()){
                        currentState = State.TURN_TO_CONE_STACK;
                        drive.followTrajectoryAsync(backCone);
                    }
                    break;
                case TURN_TO_CONE_STACK:
                    if(!drive.isBusy()){
                        currentState = State.TO_CONE_STACK;
                        drive.turnAsync(Math.toRadians(45));
                    }
                    break;
                case TO_CONE_STACK:
                    if(!drive.isBusy()){
                        if(cones == 2){
                            currentState = State.PARK;
                        }
                        else {
                            currentState = State.INTAKE;
                            robot.rightSlides.setPower(0.7);
                            robot.leftSlides.setPower(0.7);
                            robot.rightSlides.setTargetPosition(900);
                            robot.leftSlides.setTargetPosition(900);
                            drive.followTrajectoryAsync(toConeStack);
                        }
                    }
                    break;
                case INTAKE:
                    if(!drive.isBusy()){
                        robot.intake.setPower(-0.8);
                        robot.rightSlides.setPower(0.7);
                        robot.leftSlides.setPower(0.7);
                        robot.rightSlides.setTargetPosition(650);
                        robot.leftSlides.setTargetPosition(650);
                        intakeTime.reset();
                        currentState = State.LIFT_INTAKE;

                    }
                    break;
                case LIFT_INTAKE:
                    if(intakeTime.milliseconds() > 1000){
                        robot.intake.setPower(-0.2);
                        robot.rightSlides.setPower(0.9);
                        robot.leftSlides.setPower(0.9);
                        robot.rightSlides.setTargetPosition(1000);
                        robot.leftSlides.setTargetPosition(1000);
                        currentState = State.BACK_FROM_INTAKE;
                    }
                    break;
                case BACK_FROM_INTAKE:
                    if(intakeTime.milliseconds() > 2000){
                        drive.followTrajectoryAsync(backConeStack);
                        currentState = State.TURN_TO_HIGH_FROM_INTAKE;
                    }
                    break;
                case TURN_TO_HIGH_FROM_INTAKE:
                    if(!drive.isBusy()){
                        drive.turnAsync(Math.toRadians(-45));
                        robot.leftSlides.setTargetPosition(1670);
                        robot.rightSlides.setTargetPosition(1670);
                        robot.leftS.setTargetPosition(0.85);
                        robot.rightS.setTargetPosition(0.85);
                        currentState = State.TURN_TO_DROP_CONE;
                    }
                    break;
                case PARK:
                    if(!drive.isBusy()){
                        if(tagOfInterest == null || tagOfInterest.id == LEFT){
                            drive.followTrajectoryAsync(leftZone);
                            robot.rightSlides.setPower(0.7);
                            robot.leftSlides.setPower(0.7);
                            robot.leftSlides.setTargetPosition(600);
                            robot.rightSlides.setTargetPosition(600);
                            currentState = State.IDLE;
                        }
                        else if(tagOfInterest.id == RIGHT){
                            drive.followTrajectoryAsync(rightZone);
                            robot.rightSlides.setPower(0.7);
                            robot.leftSlides.setPower(0.7);
                            robot.leftSlides.setTargetPosition(600);
                            robot.rightSlides.setTargetPosition(600);
                            currentState = State.IDLE;
                        }
                        else if(tagOfInterest.id == MIDDLE){
                            drive.followTrajectoryAsync(middleZone);
                            robot.rightSlides.setPower(0.7);
                            robot.leftSlides.setPower(0.7);
                            robot.leftSlides.setTargetPosition(600);
                            robot.rightSlides.setTargetPosition(600);
                            currentState = State.IDLE;
                        }
                    }
                    break;
                case IDLE:
                    break;

            }

            drive.update();
            robot.leftS.update();
            robot.rightS.update(robot.leftS);

            Pose2d poseEstimate = drive.getPoseEstimate();

            // Print pose to telemetry
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }

        /* Actually do something useful */
//            if(tagOfInterest == null || tagOfInterest.id == LEFT){
//                //trajectory
//            }else if(tagOfInterest.id == MIDDLE){
//                //trajectory
//            }else{
//                //trajectory
//            }
    }

    public void path(){
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