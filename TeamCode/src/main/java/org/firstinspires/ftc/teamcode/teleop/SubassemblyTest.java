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
    public static boolean armTest = true;
    public static double armVel = 1;
    public static double armAccel = 1;
    public static double armProp = 1;
    public static double armIntakePos = 0;
    public static double armInit = 0.36;
    public static double armDeposit = 0.52;
    public static double armGigaIntake = 0.95;
    public static double armGigaDeposit = 0.28;


    // slides
    public static boolean slidesTest = true;
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
    public static boolean intakeTest = true;
    public boolean intakeStalled = false;
    public double intakeInPower = -.6;
    public double intakeHoldPower = -.3;
    public double intakeOutPower = .5;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        robot.arm.update();

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

    @Override
    public void loop() {
        telemetry.addLine("Gamepad 2: ");

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
