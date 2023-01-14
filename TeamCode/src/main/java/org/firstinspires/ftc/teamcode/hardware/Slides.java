package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


public class Slides {
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    public DcMotorProfiler leftSlidesProfile;
    public DcMotorProfiler rightSlidesProfile;

    public State targetPos;

    // minimum power to hold slides
    final double SLIDES_POWER = 0.8;
    final double MIN_POWER = 0.1;

    public enum State {
        HIGH(1670),
        MID(980),
        READY(600),
        LOW_AND_INTAKE(180),
        INIT(0),


        AUTO_READY(900),
        CONE_5(650);
        // the slides will never run to 0 position in opmode

        public int pos;

        State(int pos) { this.pos = pos; }

    }

    /**
     * Constructor for the Slides class.
     *
     * @param deviceManager DeviceManager object used to access the DcMotorEx objects for the slides.
     * @param afterAuto Whether the slides were powered off after an autonomous run.
     */
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
        leftSlidesProfile.setConstraints(0.6, 10);
        rightSlidesProfile.setConstraints(0.6, 10);

//        rightSlidesProfile.setTargetPosition(0);
//        leftSlidesProfile.setTargetPosition(0);

        init();
    }

    /**
     * Method for moving the slides to a certain position.
     *
     * @param s The target position for the slides.
     */
    public void run(State s){
        targetPos = s;

        // TODO uncomment to use no profile
        leftSlides.setTargetPosition(s.pos);
        rightSlides.setTargetPosition(s.pos);

//        leftSlidesProfile.setTargetPosition(s.pos);
//        rightSlidesProfile.setTargetPosition(s.pos);
    }

    /**
     * Method for moving the slides to their initial position.
     */
    public void init() {
        run(State.INIT);
    }

    /**
     * Method for moving the slides to the high position.
     */
    public void high(){
        run(State.HIGH);
    }

    /**
     * Method for moving the slides to the medium position.
     */
    public void medium(){
        run(State.MID);
    }

    /**
     * Method for moving the slides to the ready position.
     */
    public void ready(){
        run(State.READY);
    }

    /**
     * Method for moving the slides to the low and intake position.
     */
    public void lowAndIntake(){
        run(State.LOW_AND_INTAKE);
    }

    /**
     * Method for moving the slides to the Auto ready position.
     */
    public void readyAuto() {run(State.AUTO_READY);}

    /**
     * Method for moving the slides to the fifth cone position.
     */
    public void fifthAutoCone() {run(State.CONE_5);}

    /**
     * Returns the current position of the slides.
     * @return Current position of the slides.
     */
    public int getCurrentPosition() {return rightSlides.getCurrentPosition();}

    /**
     * Sets the power of the left and right slides proportional to the distance between the current position and the target position.
     * The closer the slides are to the target position, the lower the power will be.
     */
    public void setPowerProportional() {
        // largest distance = low -> high = 1670 - 180 = 1490 -> 0.9        // lower number = longer time at high power, was 1655
        double power = Range.clip(MIN_POWER + (Math.abs(targetPos.pos - getCurrentPosition()) / 1000.0),
                MIN_POWER, 1.0);

        if(targetPos.pos - getCurrentPosition() < 0){
            power = Range.clip(MIN_POWER + (Math.abs(targetPos.pos - getCurrentPosition()) / 3000.0),
                    MIN_POWER, 1.0);
            leftSlides.setPower(power);
            rightSlides.setPower(power);
        }
        else {
            leftSlides.setPower(power);
            rightSlides.setPower(power);
        }
    }
}
