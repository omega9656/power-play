package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.ServoProfiler;

@TeleOp(name="Test")
public class OmegaTeleop extends OpMode {
    Robot robot;

    enum DriveMode {
        SQUARED, CUBED, NORMAL
    }

    double fr;
    double br;
    double fl;
    double bl;

    ElapsedTime time;

    boolean intake;

    @Override
    public void init() {
        intake = false;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot = new Robot();
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

        robot.leftS.setTargetPosition(0.5);
        robot.rightS.setTargetPosition(0.5);


//        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
//        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());
//        telemetry.addData("servo direction", robot.getDirection());
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
        //moveServos();
        robot.leftS.update();
        robot.rightS.update(robot.leftS);
        intake();
        simplifiedDrive(2, DriveMode.SQUARED);
        slides();

        telemetry.addData("left slides curr", robot.leftSlides.getCurrentPosition());
        telemetry.addData("right slides curr", robot.rightSlides.getCurrentPosition());
        telemetry.addData("left slides targ", robot.leftSlides.getTargetPosition());
        telemetry.addData("right slides targ", robot.rightSlides.getTargetPosition());
        telemetry.addData("left slides targ", robot.leftSlides.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right slides targ", robot.rightSlides.getVelocity(AngleUnit.DEGREES));


        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
        telemetry.addData("right servo curr", robot.rightS.getCurrentPosition());
        telemetry.addData("left servo target", robot.leftS.getTargetPosition());
        telemetry.addData("right servo target", robot.rightS.getTargetPosition());
    }

//    public void moveServos(){
//        // intake position, no motion profile
//        if(gamepad2.b){
//            robot.leftS.setConstraints(1, 1, .75);
//            robot.rightS.setConstraints(1, 1, .75);
//            robot.leftS.setTargetPosition(0);
//            robot.rightS.setTargetPosition(0);
//        }
//
//        // hold above intake
//        if(gamepad2.a){
//            robot.leftS.setConstraints(1, 1, 1.25);
//            robot.rightS.setConstraints(1, 1, 1.25);
//            robot.leftS.setTargetPosition(0.4);
//            robot.rightS.setTargetPosition(0.4);
//        }
//
//        // outtake position, requires motion profile
//        if(gamepad2.x){
//            robot.leftS.setConstraints(1, 1, 1.25);
//            robot.rightS.setConstraints(1, 1, 1.25);
//            robot.leftS.setTargetPosition(.85);
//            robot.rightS.setTargetPosition(.85);
//        }
//    }

    public void slides(){
        if(gamepad2.dpad_up){
            robot.leftSlides.setPower(.9);
            robot.rightSlides.setPower(.9);
            robot.leftSlides.setTargetPosition(1670);
            robot.rightSlides.setTargetPosition(1670);
            robot.leftS.setTargetPosition(0.75);
            robot.rightS.setTargetPosition(0.75);
        }
        else if(gamepad2.dpad_right){
            robot.leftSlides.setPower(.9);
            robot.rightSlides.setPower(.9);
            robot.leftSlides.setTargetPosition(980);
            robot.rightSlides.setTargetPosition(980);
            robot.leftS.setTargetPosition(0.75);
            robot.rightS.setTargetPosition(0.75);
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
            robot.leftSlides.setTargetPosition(280);
            robot.rightSlides.setTargetPosition(280);
        }
        // low junction
        else if(gamepad2.dpad_left){
            robot.leftS.setTargetPosition(0.75);
            robot.rightS.setTargetPosition(0.75);
            robot.leftSlides.setPower(.7);
            robot.rightSlides.setPower(.7);
            robot.leftSlides.setTargetPosition(280);
            robot.rightSlides.setTargetPosition(280);
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

    public void simplifiedDrive(double strafe, DriveMode driveMode){
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

        robot.frontLeft.setPower(frontLeftPower*.6);
        robot.frontRight.setPower(frontRightPower*.6);
        robot.backRight.setPower(backRightPower*.6);
        robot.backLeft.setPower(backLeftPower*.6);
    }
}
