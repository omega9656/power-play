package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotOld;
import org.firstinspires.ftc.teamcode.hardware.ServoProfiler;

@Disabled
@TeleOp(name="Field Centric")
public class OmegaTeleopFieldCentric extends OpMode {
    RobotOld robot;

    enum DriveMode {
        SQUARED, CUBED, NORMAL
    }

    double fr;
    double br;
    double fl;
    double bl;

    BNO055IMU imu;
    ElapsedTime time;
    boolean intake;

    @Override
    public void init() {
        intake = false;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot = new RobotOld();

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        robot.init(hardwareMap, false, false);

        fr = robot.frontRight.getCurrentPosition();
        bl = robot.backLeft.getCurrentPosition();
        br = robot.backRight.getCurrentPosition();
        fl = robot.frontLeft.getCurrentPosition();

        robot.leftS = new ServoProfiler(robot.leftServo);
        robot.leftS.setConstraints(1, 1, 1.25);
        robot.rightS = new ServoProfiler(robot.rightServo);
        robot.rightS.setConstraints(1, 1, 1.25);

        robot.setServoPos(0);

        robot.leftS.setTargetPosition(0.45);
        robot.rightS.setTargetPosition(0.45);

        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());
        telemetry.addData("servo direction", robot.getDirection());
    }

    @Override
    public void init_loop() {

        robot.leftS.update();
        robot.rightS.update(robot.leftS);

        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
        telemetry.addData("right servo curr", robot.rightS.getCurrentPosition());
        telemetry.addData("left servo target", robot.leftS.getTargetPosition());
        telemetry.addData("right servo target", robot.rightS.getTargetPosition());
    }

    @Override
    public void start() {
        robot.rightS.setConstraints(1, 1, 1);
        robot.leftS.setConstraints(1, 1, 1);
    }

    @Override
    public void loop() {
        robot.leftS.update();
        robot.rightS.update(robot.leftS);
        intake();
        fieldCentricDrive(DriveMode.CUBED);
        slides();

        telemetry.addData("front left pos: ", robot.frontLeft.getCurrentPosition()-fl);
        telemetry.addData("front right pos: ", robot.frontRight.getCurrentPosition()-fr);
        telemetry.addData("back left pos: ", robot.backLeft.getCurrentPosition()-bl);
        telemetry.addData("back right pos: ", robot.backRight.getCurrentPosition()-br);

//        telemetry.addData("left slides curr", robot.leftSlides.getCurrentPosition());
//        telemetry.addData("right slides curr", robot.rightSlides.getCurrentPosition());
//        telemetry.addData("left slides targ", robot.leftSlides.getTargetPosition());
//        telemetry.addData("right slides targ", robot.rightSlides.getTargetPosition());
//        telemetry.addData("left slides targ", robot.leftSlides.getVelocity(AngleUnit.DEGREES));
//        telemetry.addData("right slides targ", robot.rightSlides.getVelocity(AngleUnit.DEGREES));
//
//        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
//        telemetry.addData("right servo curr", robot.rightS.getCurrentPosition());
//        telemetry.addData("left servo target", robot.leftS.getTargetPosition());
//        telemetry.addData("right servo target", robot.rightS.getTargetPosition());
    }

    public void slides(){
        if(gamepad2.dpad_up){
            robot.leftSlides.setPower(.9);
            robot.rightSlides.setPower(.9);
            robot.leftSlides.setTargetPosition(1670);
            robot.rightSlides.setTargetPosition(1670);
            robot.leftS.setTargetPosition(0.65);
            robot.rightS.setTargetPosition(0.65);
        }
        else if(gamepad2.dpad_right){
            robot.leftSlides.setPower(.9);
            robot.rightSlides.setPower(.9);
            robot.leftSlides.setTargetPosition(980);
            robot.rightSlides.setTargetPosition(980);
            robot.leftS.setTargetPosition(0.65);
            robot.rightS.setTargetPosition(0.65);
        }
        // intake position
        else if(gamepad2.dpad_down){
            robot.leftS.setTargetPosition(0);
            robot.rightS.setTargetPosition(0);
            robot.leftSlides.setPower(.7);
            robot.rightSlides.setPower(.7);
            robot.leftSlides.setTargetPosition(600);
            robot.rightSlides.setTargetPosition(600);
        }
        else if(gamepad2.x){
            robot.leftS.setTargetPosition(0);
            robot.rightS.setTargetPosition(0);
            robot.leftSlides.setPower(.7);
            robot.rightSlides.setPower(.7);
            robot.leftSlides.setTargetPosition(200);
            robot.rightSlides.setTargetPosition(200);
        }
        // low junction
        else if(gamepad2.dpad_left){
            robot.leftS.setTargetPosition(0.65);
            robot.rightS.setTargetPosition(0.65);
            robot.leftSlides.setPower(.7);
            robot.rightSlides.setPower(.7);
            robot.leftSlides.setTargetPosition(250);
            robot.rightSlides.setTargetPosition(250);
        }

    }

    public void intake(){
        if(gamepad2.right_trigger > 0.3){
            robot.intake.setPower(-0.6);
        }
        else if(gamepad2.left_trigger > 0.3){
            robot.intake.setPower(0.6);
        }
        else {
            robot.intake.setPower(0.1);
        }
    }

    public void fieldCentricDrive(DriveMode driveMode) {
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x; // used to b * 0.75 to make turning slower

        // Read inverse IMU heading, as the IMU heading is CW positive
        double offset = Math.toRadians(90); // degrees
        double botHeading = -(imu.getAngularOrientation().firstAngle + offset);

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

        robot.frontLeft.setPower(frontLeftPower * 0.6);
        robot.backLeft.setPower(backLeftPower * 0.6);
        robot.frontRight.setPower(frontRightPower * 0.6);
        robot.backRight.setPower(backRightPower * 0.6);
    }
}
