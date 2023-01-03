package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



public class Slides {
    DcMotorEx leftSlides;
    DcMotorEx rightSlides;

    DcMotorProfiler leftMotorProfile;
    DcMotorProfiler rightMotorProfile;

    State slidesPos;

    final double SLIDES_POWER = 0.9;

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

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlides.setPower(SLIDES_POWER);
        rightSlides.setPower(SLIDES_POWER);

        leftMotorProfile = new DcMotorProfiler(leftSlides);
        rightMotorProfile = new DcMotorProfiler(rightSlides);

        leftMotorProfile.setConstraints(1.2, 1.2);
        rightMotorProfile.setConstraints(1.2, 1.2);

        init();
    }

    public void run(State s){
        slidesPos = s;
        leftSlides.setTargetPosition(s.pos);
        rightSlides.setTargetPosition(s.pos);
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
