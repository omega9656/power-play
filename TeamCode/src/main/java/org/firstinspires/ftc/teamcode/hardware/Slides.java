package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class Slides {
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    public DcMotorProfiler leftSlidesProfile;
    public DcMotorProfiler rightSlidesProfile;

    public State targetPos;

    // minimum power to hold slides
    final double SLIDES_POWER = 0.9; // 0.8
    final double SLIDES_DOWN = 0.9; // 0.9
    final double MIN_POWER = 0.1;

    public enum State {
        GIGA_HIGH(2000), // 2075
        HIGH(1600), // 1670
        AUTO_HIGH(1700), // 1800
        MID(930), // 980
        READY(600), // 600
        CYCLE_READY(300), // 300
        LOW_AND_INTAKE(180), // 180
        INIT(0),


        AUTO_READY(950), // 1000
        CONE_5(575); // 650
        // the slides will never run to 0 position in opmode

        public int pos;

        State(int pos) { this.pos = pos; }

    }

    public Slides(DeviceManager deviceManager, boolean afterAuto){
        leftSlides = deviceManager.leftSlides;
        rightSlides = deviceManager.rightSlides;

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!afterAuto){
            rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(!afterAuto){
            leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // since slides are powered off after auto, and don't return to their pre-auto positions,
//        // we don't reset the encoder to maintain accuracy of deposit positions
//        // *we don't want to rezero slides position at a higher spot
//        if(!afterAuto){
//            leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }

        leftSlides.setPower(MIN_POWER); // SLIDES_POWER
        rightSlides.setPower(MIN_POWER);

        leftSlides.setTargetPosition(0);
        rightSlides.setTargetPosition(0);

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlidesProfile = new DcMotorProfiler(leftSlides);
        rightSlidesProfile = new DcMotorProfiler(rightSlides);

        // vel in proportion of max, accel in radians
        leftSlidesProfile.setConstraints(0.6, 10);
        rightSlidesProfile.setConstraints(0.6, 10);

//        rightSlidesProfile.setTargetPosition(0);
//        leftSlidesProfile.setTargetPosition(0);

        init();
    }

    public void run(State s){
        targetPos = s;

        // TODO uncomment to use no profile
        leftSlides.setTargetPosition(s.pos);
        rightSlides.setTargetPosition(s.pos);

//        leftSlidesProfile.setTargetPosition(s.pos);
//        rightSlidesProfile.setTargetPosition(s.pos);
    }

    public void init() {
        setSlidesPower(SLIDES_DOWN);
        run(State.INIT);
    }

    public void cycleReady() {
        setSlidesPower(SLIDES_DOWN);
        run(State.CYCLE_READY);}

    public void gigaHigh() {
        setSlidesPower(SLIDES_POWER);
        run(State.GIGA_HIGH);}

    public void high(){
        setSlidesPower(SLIDES_POWER);
        run(State.HIGH);
    }

    public void medium(){
        setSlidesPower(SLIDES_POWER);
        run(State.MID);
    }

    public void ready(){
        setSlidesPower(SLIDES_DOWN);
        run(State.READY);
    }

    public void lowAndIntake(){
        setSlidesPower(SLIDES_DOWN);
        run(State.LOW_AND_INTAKE);
    }

    public void readyAuto() {
        setSlidesPower(SLIDES_DOWN);
        run(State.AUTO_READY);}

    public void fifthAutoCone() {
        setSlidesPower(SLIDES_DOWN);
        run(State.CONE_5);}

    public void autoHigh() {
        setSlidesPower(SLIDES_POWER);
        run(State.AUTO_HIGH);
    }

    public int getCurrentPosition() {return rightSlides.getCurrentPosition();}

    // uses p loop to set power
    public void setPowerProportional() {
        // largest distance = low -> high = 1670 - 180 = 1490 -> 0.9        // lower number = longer time at high power, was 1655
        double power = Range.clip(MIN_POWER + (Math.abs(targetPos.pos - getCurrentPosition()) / 1000.0),
                MIN_POWER, 1.0);

        if(targetPos.pos - getCurrentPosition() < 0){
//            power = Range.clip(MIN_POWER + (Math.abs(targetPos.pos - getCurrentPosition()) / 1000.0),
//                    MIN_POWER, 1.0);
            leftSlides.setPower(0.8);
            rightSlides.setPower(0.8);
        }
        else {
            leftSlides.setPower(power);
            rightSlides.setPower(power);
        }
    }

    public void setVelocityProportional(){
        double MIN_VEL = 0.5 * Math.PI; // 1.74
        double MAX_VEL = 8 * Math.PI; // 24
        double vel = Range.clip(MIN_VEL + (Math.abs(targetPos.pos - getCurrentPosition()) / 75.0),
                MIN_VEL, MAX_VEL);
        // down
        if(targetPos.pos - getCurrentPosition() < 0){
            leftSlides.setVelocity(-vel, AngleUnit.RADIANS);
            rightSlides.setVelocity(-vel, AngleUnit.RADIANS);
        }
        // up
        else {
            leftSlides.setVelocity(vel, AngleUnit.RADIANS);
            rightSlides.setVelocity(vel, AngleUnit.RADIANS);
        }
    }

    public void setSlidesPower(double power){
        leftSlides.setPower(power);
        rightSlides.setPower(power);
    }
}
