package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


public class Slides {
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    public DcMotorProfiler leftSlidesProfile;
    public DcMotorProfiler rightSlidesProfile;

    State slidesPos;

    // minimum power to hold slides
    final double SLIDES_POWER = 0.1;

    enum State {
        HIGH(1670),
        MID(980),
        READY(600),
        LOW_AND_INTAKE(280),
        INIT(0);
        // the slides will never run to 0 position in opmode

        public int pos;

        State(int pos) { this.pos = pos; }

    }

    public Slides(DeviceManager deviceManager, boolean afterAuto){
        leftSlides = deviceManager.leftSlides;
        rightSlides = deviceManager.rightSlides;

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // since slides are powered off after auto, and don't return to their pre-auto positions,
        // we don't reset the encoder to maintain accuracy of deposit positions
        // *we don't want to rezero slides position at a higher spot
        if(!afterAuto){
            leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        leftSlides.setPower(SLIDES_POWER);
        rightSlides.setPower(SLIDES_POWER);

        leftSlides.setTargetPosition(0);
        rightSlides.setTargetPosition(0);

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlidesProfile = new DcMotorProfiler(leftSlides);
        rightSlidesProfile = new DcMotorProfiler(rightSlides);

        // vel in proportion of max, accel in radians
        leftSlidesProfile.setConstraints(0.7, 5);
        rightSlidesProfile.setConstraints(0.7, 5);

        init();
    }

    public void run(State s){
        slidesPos = s;

        // TODO uncomment to use no profile
//        leftSlides.setTargetPosition(s.pos);
//        rightSlides.setTargetPosition(s.pos);

        leftSlidesProfile.setTargetPosition(s.pos);
        rightSlidesProfile.setTargetPosition(s.pos);
    }

    public void init() {
        run(State.INIT);
    }

    public void high(){
        run(State.HIGH);
    }

    public void medium(){
        run(State.MID);
    }

    public void ready(){
        run(State.READY);
    }

    public void lowAndIntake(){
        run(State.LOW_AND_INTAKE);
    }
}
