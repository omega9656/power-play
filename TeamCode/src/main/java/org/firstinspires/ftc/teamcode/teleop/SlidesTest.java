package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.ServoProfiler;

@Disabled
public class SlidesTest extends OpMode {
    Robot robot;

    @Override
    public void init() {
        robot = new Robot();
        robot.init(hardwareMap, false, false);
    }

    @Override
    public void loop() {
        slides();

        telemetry.addData("left slides curr", robot.leftSlides.getCurrentPosition());
        telemetry.addData("right slides curr", robot.rightSlides.getCurrentPosition());
        telemetry.addData("left slides targ", robot.leftSlides.getTargetPosition());
        telemetry.addData("right slides targ", robot.rightSlides.getTargetPosition());
        telemetry.addData("left slides targ", robot.leftSlides.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("right slides targ", robot.rightSlides.getVelocity(AngleUnit.DEGREES));
    }

    public void slides(){
        if(gamepad2.dpad_up){
            robot.leftSlides.setTargetPosition(1000);
            robot.rightSlides.setTargetPosition(1000);
        }
        else if(gamepad2.dpad_left){
            robot.leftSlides.setTargetPosition(1500);
            robot.rightSlides.setTargetPosition(1500);
        }
        else if(gamepad2.dpad_down){
            robot.leftSlides.setTargetPosition(0);
            robot.rightSlides.setTargetPosition(0);
        }
    }

}
