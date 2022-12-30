package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class OmegaTeleopModular extends OpMode {

    Robot robot;
    ElapsedTime time;
    BNO055IMU imu;

    boolean fieldCentric;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, true);

        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // for field centric option
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        fieldCentric = false;
    }

    @Override
    public void init_loop() {
        if(gamepad1.a){
            fieldCentric = !fieldCentric;
        }

        telemetry.addData("field centric: ", fieldCentric);
        telemetry.update();
    }

    @Override
    public void loop() {
        if(fieldCentric){
            fieldCentricDrive(OmegaTeleopFieldCentric.DriveMode.CUBED);
        }
        else {
            drive(2, OmegaTeleop.DriveMode.CUBED);
        }

        deposit();
        intake();

        telemetry.addData("field centric: ", fieldCentric);
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
        robot.intake.hold();
        // in
        if(gamepad2.right_trigger > 0.3){
            robot.intake.in();
        }
        // out
        if(gamepad2.left_trigger > 0.3){
            robot.intake.stop();
        }
    }

    public void drive(double strafe, OmegaTeleop.DriveMode driveMode){
        double vertical = -gamepad1.left_stick_y;  // flip sign because y axis is reversed on joystick

        // moving left joystick to the right means robot moves right
        double horizontal = gamepad1.left_stick_x * strafe;  // counteract imperfect strafing by multiplying by constant

        // moving right joystick to the right means clockwise rotation of robot
        double rotate = gamepad1.right_stick_x * 0.75;

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
        if (driveMode == OmegaTeleop.DriveMode.SQUARED) {
            // need to keep the sign, so multiply by absolute value of itself
            frontLeftPower *= Math.abs(frontLeftPower);
            backLeftPower *= Math.abs(backLeftPower);
            frontRightPower *= Math.abs(frontRightPower);
            backRightPower *= Math.abs(backRightPower);
        } else if (driveMode == OmegaTeleop.DriveMode.CUBED) {
            frontLeftPower = Math.pow(frontLeftPower, 3);
            backLeftPower = Math.pow(backLeftPower, 3);
            frontRightPower = Math.pow(frontRightPower, 3);
            backRightPower = Math.pow(backRightPower, 3);
        } // if drive mode is normal, don't do anything

        robot.drivetrain.frontLeft.setPower(frontLeftPower*.6);
        robot.drivetrain.frontRight.setPower(frontRightPower*.6);
        robot.drivetrain.backRight.setPower(backRightPower*.6);
        robot.drivetrain.backLeft.setPower(backLeftPower*.6);
    }

    public void fieldCentricDrive(OmegaTeleopFieldCentric.DriveMode driveMode) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x * 0.75; // makes turning slower

        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle;

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

        if (driveMode == OmegaTeleopFieldCentric.DriveMode.SQUARED) {
            // need to keep the sign, so multiply by absolute value of itself
            frontLeftPower *= Math.abs(frontLeftPower);
            backLeftPower *= Math.abs(backLeftPower);
            frontRightPower *= Math.abs(frontRightPower);
            backRightPower *= Math.abs(backRightPower);
        } else if (driveMode == OmegaTeleopFieldCentric.DriveMode.CUBED) {
            frontLeftPower = Math.pow(frontLeftPower, 3);
            backLeftPower = Math.pow(backLeftPower, 3);
            frontRightPower = Math.pow(frontRightPower, 3);
            backRightPower = Math.pow(backRightPower, 3);
        } // if drive mode is normal, don't do anything

        robot.drivetrain.frontLeft.setPower(frontLeftPower * 0.6);
        robot.drivetrain.backLeft.setPower(backLeftPower * 0.6);
        robot.drivetrain.frontRight.setPower(frontRightPower * 0.6);
        robot.drivetrain.backRight.setPower(backRightPower * 0.6);
    }
}
