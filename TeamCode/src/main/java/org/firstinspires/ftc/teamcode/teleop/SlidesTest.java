package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp
public class SlidesTest extends OpMode {

    Robot robot;
    ElapsedTime time;
    double distance;

    // https://www.gobilda.com/content/spec_sheets/5204-8002-0014_spec_sheet.pdf
    public final double TICKS_PER_REVOLUTION = 384.5;  // encoder countable events per revolution (output shaft)
    public final int MAX_RPM = 435;  // *no-load* speed @ 12VDC
    public final double MAX_TICKS_PER_SEC = MAX_RPM * TICKS_PER_REVOLUTION / 60;  // 2787 ticks/sec
    public final double MAX_RADIANS_PER_SEC = ticksToRadians(MAX_TICKS_PER_SEC); // 45.55 rad/sec

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);
        time = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        distance = 0;
    }

    @Override
    public void loop() {
        slides();

//        double kP = 0.3;
//
//        distance = ticksToRadians(robot.slides.targetPos.pos - robot.slides.getCurrentPosition());
//        double instantTargetPos = robot.slides.rightSlidesProfile.motionProfile(time.seconds(), distance, robot.slides.leftSlidesProfile.getMaxVel());
//        double power = (instantTargetPos - robot.slides.rightSlides.getCurrentPosition()) * kP;
//        robot.slides.rightSlides.setPower(power);
//        robot.slides.leftSlides.setPower(power);

        robot.slides.setPowerProportional();

        telemetry.addData("slides power ", robot.slides.leftSlides.getPower());
        telemetry.addData("slides pos", robot.slides.getCurrentPosition());
        telemetry.addData("slides target pos", robot.slides.targetPos.pos);
        telemetry.update();
    }

    public void slides(){
        if(gamepad2.dpad_up){
            time.reset();
            robot.slides.high();
        }
        if(gamepad2.dpad_right){
            time.reset();
            robot.slides.medium();
        }
        if(gamepad2.dpad_left){
            time.reset();
            robot.slides.lowAndIntake();
        }
        if(gamepad2.dpad_down){
            time.reset();
            robot.slides.ready();
        }
        if(gamepad2.x){
            time.reset();
            robot.slides.lowAndIntake();
        }
    }

    public double ticksToRadians(double ticks) {return 2 * Math.PI * ticks / TICKS_PER_REVOLUTION;}
}
