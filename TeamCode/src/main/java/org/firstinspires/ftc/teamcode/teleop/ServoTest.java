package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp(name = "Giga TeleOp Test")
public class ServoTest extends OpMode {
    Robot robot;

    enum DriveMode {
        SQUARED, CUBED, NORMAL
    }

    ElapsedTime time;

    boolean intake;

    @Override
    public void init() {
        intake = false;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        robot.arm.init();

//        robot.leftS.setTargetPosition(0.5);
//        robot.rightS.setTargetPosition(0.5);


//        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
//        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());
//        telemetry.addData("servo direction", robot.getDirection());
    }

    @Override
    public void init_loop() {

        robot.arm.leftServoProfile.update();
        robot.arm.rightServoProfile.update();

        telemetry.addData("slides ", robot.slides.getCurrentPosition());
        telemetry.addData("slides targ", robot.slides.targetPos.pos);

        telemetry.addData("left servo curr", robot.arm.leftServo.getPosition());
        telemetry.addData("right servo curr", robot.arm.rightServo.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.arm.leftServoProfile.update();
        robot.arm.rightServoProfile.update();

        //moveServos();
        intake();
        extendoLift();
        dtStraight();

        telemetry.addData("slides ", robot.slides.getCurrentPosition());
        telemetry.addData("slides targ", robot.slides.targetPos.pos);

        telemetry.addData("left servo curr", robot.arm.leftServo.getPosition());
        telemetry.addData("right servo curr", robot.arm.rightServo.getPosition());
        telemetry.update();
    }

    public void dtStraight(){
        if(gamepad1.right_bumper){
            robot.drivetrain.frontLeft.setPower(0.2);
            robot.drivetrain.frontRight.setPower(0.2);
            robot.drivetrain.backLeft.setPower(0.2);
            robot.drivetrain.backRight.setPower(0.2);
        }
        if(gamepad1.left_bumper){
            robot.drivetrain.frontLeft.setPower(-0.2);
            robot.drivetrain.frontRight.setPower(-0.2);
            robot.drivetrain.backLeft.setPower(-0.2);
            robot.drivetrain.backRight.setPower(-0.2);
        }
        robot.drivetrain.frontLeft.setPower(0);
        robot.drivetrain.frontRight.setPower(0);
        robot.drivetrain.backLeft.setPower(0);
        robot.drivetrain.backRight.setPower(0);
    }

    public void intake(){
        robot.intake.hold();
        // in
        if(gamepad2.right_trigger > 0.3){
            robot.intake.in();
        }
        // out
        if(gamepad2.left_trigger > 0.3){
            robot.intake.out();
        }
    }

    public void extendoLift(){
        if(gamepad2.right_bumper){
            robot.slides.gigaHigh();
            robot.arm.init();
        }

        if(gamepad2.left_bumper){
            robot.slides.cycleReady();
            robot.arm.extendoIntake();
        }

        if(gamepad2.b){
            robot.slides.init();
            robot.arm.extendoIntake();
        }
    }
}
