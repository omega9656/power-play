package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.Arrays;

@Config
@TeleOp
public class OmegaTeleopModular extends OpMode {
    enum DriveMode {
        SQUARED, CUBED, NORMAL
    }

    Robot robot;
    ElapsedTime time;
    BNO055IMU imu;

    // default, in degrees
    public static double startHeading = 360;

    public static boolean fieldCentric = true;
    boolean slidesProfile = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // for field centric option
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        robot.arm.init();
    }

    @Override
    public void init_loop() {

        robot.arm.update();

        telemetry.addData("start heading", startHeading);
        telemetry.addData("is arm going upwards", robot.arm.isGoingUpwards);
        telemetry.addData("target pos", robot.arm.armPosition);
        telemetry.addData("arm constraints vel, accel", Arrays.toString(robot.arm.getConstraints()));

        telemetry.addData("left servo pos: ", robot.arm.leftServoProfile.getCurrentPosition());
        telemetry.addData("left servo targ: ", robot.arm.leftServoProfile.getTargetPosition());

        telemetry.update();
    }

    @Override
    public void loop() {
        if(fieldCentric){
            fieldCentricDrive(DriveMode.CUBED);
        }
        else {
            drive(2, DriveMode.CUBED);
        }

        robot.slides.setPowerProportional();
        //robot.slides.setSlidesPower(1);

        robot.arm.update();
        deposit();
        intake();
        extendoLift();

        telemetry.addData("is arm going upwards", robot.arm.isGoingUpwards);
        telemetry.addData("target pos", robot.arm.armPosition);
        telemetry.addData("arm constraints vel, accel", Arrays.toString(robot.arm.getConstraints()));

        telemetry.addData("slides ", robot.slides.getCurrentPosition());
        telemetry.addData("slides targ", robot.slides.targetPos.pos);
        telemetry.addData("slides pow", robot.slides.leftSlides.getPower());

        telemetry.addData("left servo pos: ", robot.arm.getCurrentPosition());
        telemetry.addData("left servo targ: ", robot.arm.getTargetPosition());

        telemetry.addData("front left: ", robot.drivetrain.frontLeft.getPower());
        telemetry.addData("front right: ", robot.drivetrain.frontRight.getPower());
        telemetry.addData("back left: ", robot.drivetrain.backLeft.getPower());
        telemetry.addData("back right: ", robot.drivetrain.backRight.getPower());

        telemetry.update();
    }

    public void deposit(){
        // high junction
        if(gamepad2.dpad_up){
            robot.arm.deposit();
            robot.slides.high();
        }
        // med junction
        if(gamepad2.dpad_right){
            robot.arm.deposit();
            robot.slides.medium();
        }
        // low junction
        if(gamepad2.dpad_left) {
            robot.arm.deposit();
            robot.slides.lowAndIntake();
        }
        // ready intake pos, above low junction
        if(gamepad2.dpad_down){
            robot.arm.intake();
            robot.slides.ready();
        }
        // intake
        if(gamepad2.x){
            robot.arm.intake();
            robot.slides.lowAndIntake();
        }
    }

    public void intake(){
        // in; buffer point
        if(gamepad2.right_trigger > 0.3){
            robot.intake.in();
        }
        // out
        else if(gamepad2.left_trigger > 0.1){
            robot.intake.out();
        }
        else {
            robot.intake.telehold();
        }
    }

    public void extendoLift(){
        // outtake
        if(gamepad2.right_bumper){
            robot.slides.gigaHigh();
            robot.arm.extendoDeposit();
        }
        // above intake
        if(gamepad2.left_bumper){
            robot.slides.cycleReady();
            robot.arm.extendoIntake();
        }
        //intake
        if(gamepad2.b){
            robot.slides.init();
            robot.arm.extendoIntake();
        }
    }

    public void fieldCentricDrive(DriveMode driveMode) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.8; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x ; //  * 0.75

        // robot ends up in different orientation after auto
        double offset = Math.toRadians(startHeading-90); // degrees
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + offset);

        telemetry.addData("heading", botHeading);

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        if (driveMode == DriveMode.SQUARED) {
            // need to keep the sign, so multiply by absolute value of itself
            frontLeftPower *= Math.abs(frontLeftPower);
            backLeftPower *= Math.abs(backLeftPower);
            frontRightPower *= Math.abs(frontRightPower);
            backRightPower *= Math.abs(backRightPower);
        } else if (driveMode == DriveMode.CUBED) {
            frontLeftPower = Math.pow(frontLeftPower, 3);
            backLeftPower = Math.pow(backLeftPower, 3);
            frontRightPower = Math.pow(frontRightPower, 3);
            backRightPower = Math.pow(backRightPower, 3);
        } // if drive mode is normal, don't do anything

        robot.drivetrain.frontLeft.setPower(frontLeftPower);
        robot.drivetrain.backLeft.setPower(backLeftPower);
        robot.drivetrain.frontRight.setPower(frontRightPower);
        robot.drivetrain.backRight.setPower(backRightPower);
    }

    public void drive(double strafe, DriveMode driveMode){
        double vertical = -gamepad1.left_stick_y;  // flip sign because y axis is reversed on joystick

        // moving left joystick to the right means robot moves right
        double horizontal = gamepad1.left_stick_x * strafe;  // counteract imperfect strafing by multiplying by constant

        // moving right joystick to the right means clockwise rotation of robot
        double rotate = gamepad1.right_stick_x; //  * 0.75

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(vertical) + Math.abs(horizontal) + Math.abs(rotate), 1);

        // calculate initial power from gamepad inputs
        // to understand this, draw force vector diagrams (break into components)
        // and observe the goBILDA diagram on the GM0 page (linked above)
        double frontLeftPower = (vertical + horizontal + rotate) / denominator;
        double backLeftPower = (vertical - horizontal + rotate) / denominator;
        double frontRightPower = (vertical - horizontal - rotate) / denominator; // - hor good
        double backRightPower = (vertical + horizontal - rotate) / denominator; // + hor good

        // square or cube gamepad inputs
        if (driveMode == DriveMode.SQUARED) {
            // need to keep the sign, so multiply by absolute value of itself
            frontLeftPower *= Math.abs(frontLeftPower);
            backLeftPower *= Math.abs(backLeftPower);
            frontRightPower *= Math.abs(frontRightPower);
            backRightPower *= Math.abs(backRightPower);
        } else if (driveMode == DriveMode.CUBED) {
            frontLeftPower = Math.pow(frontLeftPower, 3);
            backLeftPower = Math.pow(backLeftPower, 3);
            frontRightPower = Math.pow(frontRightPower, 3);
            backRightPower = Math.pow(backRightPower, 3);
        } // if drive mode is normal, don't do anything

        if(gamepad1.right_trigger > 0.4){
            robot.drivetrain.frontLeft.setPower(frontLeftPower);
            robot.drivetrain.frontRight.setPower(frontRightPower);
            robot.drivetrain.backRight.setPower(backRightPower);
            robot.drivetrain.backLeft.setPower(backLeftPower);
        }
        else {
            robot.drivetrain.frontLeft.setPower(frontLeftPower);
            robot.drivetrain.frontRight.setPower(frontRightPower);
            robot.drivetrain.backRight.setPower(backRightPower);
            robot.drivetrain.backLeft.setPower(backLeftPower);
        }
    }
}
