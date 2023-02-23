package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class SubassemblyTest extends OpMode {
    Robot robot;

    // arm
    public static boolean armTest = false;
    public static double armVel = 1;
    public static double armAccel = 1;
    public static double armProp = 1;
    public static double armIntakePos = 0;
    public static double armInit = 0.36;
    public static double armDeposit = 0.52;
    public static double armGigaIntake = 0.95;
    public static double armGigaDeposit = 0.28;


    // slides
    public static boolean slidesTest = false;
    public static int slidesHigh = 1600;
    public static int slidesMid = 930;
    public static int slidesReady = 600;
    public static int slidesLowAndIntake = 180;
    public static int slidesGigaDeposit = 2050;
    public static int slidesPower = 1;

    public static int autoCone5 = 0;
    public static int autoCone4 = 0;
    public static int autoCone3 = 0;
    public static int autoCone2 = 0;

    // intake
    public static boolean intakeTest = false;
    public boolean intakeStalled = false;
    public double intakeInPower = -.6;
    public double intakeHoldPower = -.3;
    public double intakeOutPower = .5;

    public static double pow = 0.058823530375957486;

    // dt
    public static boolean dtTest = true;
    public static boolean dtStraightPower = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        if(armTest){
            robot.arm.update();
        }

        telemetry.addData("current arm pos: ", robot.arm.getCurrentPosition());
        telemetry.addData("target arm pos: ", robot.arm.getTargetPosition());
        telemetry.update();
    }

    public void moveLift(){
        // high
        if(gamepad2.dpad_up){
            robot.slides.setTargetPos(slidesHigh);
            robot.arm.setArmPosition(armDeposit);
        }
        // mid
        if(gamepad2.dpad_right){
            robot.slides.setTargetPos(slidesMid);
            robot.arm.setArmPosition(armDeposit);
        }
        // low
        if(gamepad2.dpad_left){
            robot.slides.setTargetPos(slidesLowAndIntake);
            robot.arm.setArmPosition(armDeposit);
        }
        // ready
        if(gamepad2.dpad_down){
            robot.slides.setTargetPos(slidesReady);
            robot.arm.setArmPosition(armIntakePos);
        }
        // intake
        if(gamepad2.x){
            robot.slides.setTargetPos(slidesLowAndIntake);
            robot.arm.setArmPosition(armIntakePos);
        }
        // giga intake ready
        if(gamepad2.left_bumper){
            robot.slides.setTargetPos(slidesLowAndIntake + 500);
            robot.arm.setArmPosition(armGigaIntake);
        }
        // giga intake
        if(gamepad2.b){
            robot.slides.setTargetPos(slidesLowAndIntake);
            robot.arm.setArmPosition(armGigaIntake);
        }
        if(gamepad2.right_bumper){
            robot.slides.setTargetPos(slidesGigaDeposit);
            robot.arm.setArmPosition(armGigaDeposit);
        }
    }

    public void intake(){
        //hold
        if(gamepad2.left_bumper || robot.intake.isStalled()){
            robot.intake.setPower(intakeHoldPower);
        }
        //outtake
        else if(gamepad2.left_trigger > 0.3){
            robot.intake.setPower(intakeOutPower);
        }
        //intake
        else if(gamepad2.right_trigger > 0.3){
            robot.intake.setPower(intakeInPower);
        }
        else {
            robot.intake.setPower(0);
        }
    }

    public void straight(){
//        if(gamepad2.x){
//            robot.drivetrain.frontLeft.setPower(pow);
//            robot.drivetrain.frontRight.setPower(pow);
//            robot.drivetrain.backRight.setPower(pow);
//            robot.drivetrain.backLeft.setPower(pow);
//        }
//        else if(gamepad2.y){
//            robot.drivetrain.frontLeft.setPower(0);
//            robot.drivetrain.frontRight.setPower(0);
//            robot.drivetrain.backRight.setPower(0);
//            robot.drivetrain.backLeft.setPower(0);
//        }
        if(gamepad2.right_trigger >= 0) {
            double power = gamepad2.right_trigger;
            robot.drivetrain.frontLeft.setPower(power*.6);
            robot.drivetrain.frontRight.setPower(power*.6);
            robot.drivetrain.backRight.setPower(power*.6);
            robot.drivetrain.backLeft.setPower(power*.6);
        }

    }

    public void drive(double strafe, OmegaTeleopModular.DriveMode driveMode){
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
        if (driveMode == OmegaTeleopModular.DriveMode.SQUARED) {
            // need to keep the sign, so multiply by absolute value of itself
            frontLeftPower *= Math.abs(frontLeftPower);
            backLeftPower *= Math.abs(backLeftPower);
            frontRightPower *= Math.abs(frontRightPower);
            backRightPower *= Math.abs(backRightPower);
        } else if (driveMode == OmegaTeleopModular.DriveMode.CUBED) {
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
            robot.drivetrain.frontLeft.setPower(frontLeftPower*.6);
            robot.drivetrain.frontRight.setPower(frontRightPower*.6);
            robot.drivetrain.backRight.setPower(backRightPower*.6);
            robot.drivetrain.backLeft.setPower(backLeftPower*.6);
        }
    }

    @Override
    public void loop() {
        telemetry.addLine("Gamepad 2: ");

        if(dtStraightPower){
            robot.drivetrain.frontLeft.setPower(pow);
            robot.drivetrain.frontRight.setPower(pow);
            robot.drivetrain.backRight.setPower(pow);
            robot.drivetrain.backLeft.setPower(pow);
        }

        if(dtTest){
            //drive(2, OmegaTeleopModular.DriveMode.CUBED);
            straight();

            telemetry.addData("front left: ", robot.drivetrain.frontLeft.getPower());
            telemetry.addData("front right: ", robot.drivetrain.frontRight.getPower());
            telemetry.addData("back left: ", robot.drivetrain.backLeft.getPower());
            telemetry.addData("back right: ", robot.drivetrain.backRight.getPower());
        }

        if(intakeTest){
            intake();

            telemetry.addLine("         Right trigger: intake ");
            telemetry.addLine("         Left trigger: outtake ");
            telemetry.addLine("         Left bumper: hold ");

            telemetry.addData("Is the intake motor stalled? ", intakeStalled);
            telemetry.addData("current intake power: ", robot.intake.getPower());
        }

        if(slidesTest && armTest){
            robot.arm.setConstraints(armVel, armAccel, armProp);
            robot.arm.update();

            robot.slides.setSlidesPower(slidesPower);
            moveLift();

            telemetry.addLine("         dpad up: high ");
            telemetry.addLine("         dpad right: medium ");
            telemetry.addLine("         dpad left: low ");
            telemetry.addLine("         dpad low: ready ");

            telemetry.addData("slides power", robot.slides.leftSlides.getPower());
            telemetry.addData("current arm pos: ", robot.arm.getCurrentPosition());
            telemetry.addData("target arm pos: ", robot.arm.getTargetPosition());
        }


        telemetry.update();
    }
}
