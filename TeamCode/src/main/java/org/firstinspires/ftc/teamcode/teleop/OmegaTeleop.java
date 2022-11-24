package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

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

    boolean motionProfile = false;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, false);

        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());

//        fr = robot.frontRight.getCurrentPosition();
//        bl = robot.backLeft.getCurrentPosition();
//        br = robot.backRight.getCurrentPosition();
//        fl = robot.frontLeft.getCurrentPosition();
    }

    @Override
    public void loop() {
        compareServos();
        if(motionProfile){
            robot.leftS.update();
            robot.rightS.update(robot.leftS);
        }
        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());
        //intake();
        //moveServos();
        //simplifiedDrive(2);

//        telemetry.addData("front right pos", robot.frontRight.getCurrentPosition()-fr);
//        telemetry.addData("back right pos", robot.backRight.getCurrentPosition()-br);
//        telemetry.addData("front left pos", robot.frontLeft.getCurrentPosition()-fl);
//        telemetry.addData("back left pos", robot.backLeft.getCurrentPosition()-bl);
    }

    public void compareServos(){
        /*
        if(gamepad1.a){
            robot.leftS.setTargetPosition(1);
            robot.rightS.setTargetPosition(1);
        }
        if (gamepad1.b){
            robot.leftS.setTargetPosition(0);
            robot.rightS.setTargetPosition(0);
        }*/

        // init position, no motion profile
        if(gamepad1.b){
            motionProfile = false;
            robot.leftS.setTargetPosition(0);
            robot.rightS.setTargetPosition(0);
        }

        // intake position, requires motion profile
        if(gamepad1.a){
            motionProfile = true;
            // TODO, when moving from outtake to intake position, what pos is servo in originally? it'll technically b negative
            if(robot.getDirection() == Robot.ServoDirection.OUTTAKE) robot.flipDirection();
            robot.leftS.setTargetPosition(0.2);
            robot.leftS.setTargetPosition(0.2);
        }

        // outtake position, requires motion profile
        if(gamepad1.x){
            motionProfile = true;
            if(robot.getDirection() == Robot.ServoDirection.INTAKE) robot.flipDirection();
            robot.leftS.setTargetPosition(0.2);
            robot.leftS.setTargetPosition(0.2);
        }
    }

    public void moveServos(){
        // intake pos
        if(gamepad1.a){
            if(robot.getDirection() == Robot.ServoDirection.OUTTAKE) robot.flipDirection();
            robot.setServoPos(0.2);
        }
        // 0 pos
        if(gamepad1.b) robot.setServoPos(0);

        // outtake pos
        if(gamepad1.x){
            if(robot.getDirection() == Robot.ServoDirection.INTAKE) robot.flipDirection();
            robot.setServoPos(0.2);
        }
    }

    public void intake(){
        if(gamepad1.right_trigger > 0.3){
            robot.intake.setVelocity(720*9, AngleUnit.DEGREES);
        }
        else if(gamepad1.left_trigger > 0.3){
            robot.intake.setVelocity(-720*9, AngleUnit.DEGREES);
        }
        else {
            robot.intake.setVelocity(0);
        }
    }

    public void simplifiedDrive(double strafe){
        double vertical = -gamepad1.left_stick_y;  // flip sign because y axis is reversed on joystick

        // moving left joystick to the right means robot moves right
        double horizontal = gamepad1.left_stick_x * strafe;  // counteract imperfect strafing by multiplying by constant

        // moving right joystick to the right means clockwise rotation of robot
        double rotate = gamepad1.right_stick_x;

        // calculate initial power from gamepad inputs
        // to understand this, draw force vector diagrams (break into components)
        // and observe the goBILDA diagram on the GM0 page (linked above)
        double frontLeftPower = vertical + horizontal + rotate;
        double backLeftPower = vertical - horizontal + rotate;
        double frontRightPower = vertical - horizontal - rotate;
        double backRightPower = vertical + horizontal - rotate;

        robot.frontLeft.setPower(frontLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);
        robot.backLeft.setPower(backLeftPower);
    }

    public void driveL(double strafe, DriveMode driveMode) {
        // https://gm0.copperforge.cc/en/stable/docs/software/mecanum-drive.html
        // https://www.chiefdelphi.com/t/paper-mecanum-and-omni-kinematic-and-force-analysis/106153/5 (3rd paper)

        // moving left joystick up means robot moves forward
        double vertical = -gamepad1.left_stick_y;  // flip sign because y axis is reversed on joystick

        // moving left joystick to the right means robot moves right
        double horizontal = gamepad1.left_stick_x * strafe;  // counteract imperfect strafing by multiplying by constant

        // moving right joystick to the right means clockwise rotation of robot
        double rotate = gamepad1.right_stick_x;

        // calculate initial power from gamepad inputs
        // to understand this, draw force vector diagrams (break into components)
        // and observe the goBILDA diagram on the GM0 page (linked above)
        double frontLeftPower = vertical + horizontal + rotate;
        double backLeftPower = vertical - horizontal + rotate;
        double frontRightPower = vertical - horizontal - rotate;
        double backRightPower = vertical + horizontal - rotate;

        telemetry.addData("fl pow", frontLeftPower);
        telemetry.addData("fr pow", frontRightPower);
        telemetry.addData("bl pow", backLeftPower);
        telemetry.addData("br pow", backRightPower);

        // if there is a power level that is out of range
        if (
                Math.abs(frontLeftPower) > 1 ||
                        Math.abs(backLeftPower) > 1 ||
                        Math.abs(frontRightPower) > 1 ||
                        Math.abs(backRightPower) > 1
        ) {
            // scale the power within [-1, 1] to keep the power levels proportional
            // (if the power is over 1 the FTC SDK will just make it 1)

            // find the largest power
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
            max = Math.max(Math.abs(frontRightPower), max);
            max = Math.max(Math.abs(backRightPower), max);

            // scale everything with the ratio max:1
            // don't need to worry about signs because max is positive
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

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

        // set final power values to motors
        robot.frontLeft.setPower(frontLeftPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);
    }
}
