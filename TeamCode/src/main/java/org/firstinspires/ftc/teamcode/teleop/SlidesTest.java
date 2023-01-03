package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotOld;

@TeleOp
public class SlidesTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);
    }

    @Override
    public void loop() {
        slides();
        robot.slides.rightSlidesProfile.update();
        robot.slides.leftSlidesProfile.update();

        telemetry.addData("left slides velocity", robot.slides.leftSlides.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("right slides velocity", robot.slides.rightSlides.getVelocity(AngleUnit.RADIANS));
        telemetry.addData("left slides curr", robot.slides.leftSlides.getCurrentPosition());
        telemetry.addData("right slides curr", robot.slides.rightSlides.getCurrentPosition());
        telemetry.addData("left slides targ", robot.slides.leftSlides.getTargetPosition());
        telemetry.addData("right slides targ", robot.slides.rightSlides.getTargetPosition());
        telemetry.update();
    }

    public void slides(){
        if(gamepad2.dpad_up){
            robot.slides.high();
        }
        if(gamepad2.dpad_right){
            robot.slides.medium();
        }
        if(gamepad2.dpad_left){
            robot.slides.lowAndIntake();
        }
        if(gamepad2.dpad_down){
            robot.slides.ready();
        }
        if(gamepad2.x){
            robot.slides.lowAndIntake();
        }
    }

}
