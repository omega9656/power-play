package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Slides {
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    public State targetPos;

    final double MIN_POWER = 0.00625;

    final double proportional = 1 / 1000.0;

    public enum State {
        GIGA_HIGH(2100), // 2075
        AUTO_HIGH(1860),
        HIGH(1650), // 1700
        MID(980), // 1030
        READY(700), // 600
        CYCLE_READY(400), // 300
        LOW_AND_INTAKE(220), // 180
        INIT(0),


        AUTO_READY(950), // 1000
        CONE_5(575); // 650
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

        leftSlides.setPower(MIN_POWER);
        rightSlides.setPower(MIN_POWER);

        leftSlides.setTargetPosition(0);
        rightSlides.setTargetPosition(0);

        rightSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlides.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        init();
    }

    /**
     * Method for moving the slides to a certain position.
     *
     * @param s The target position for the slides.
     */
    public void run(State s){
        targetPos = s;

        leftSlides.setTargetPosition(s.pos);
        rightSlides.setTargetPosition(s.pos);
    }

    /**
     * sets positions to motors w/o state machines, for testing
     * @param pos target position for the slides
     */
    public void setTargetPos(int pos){
        leftSlides.setTargetPosition(pos);
        rightSlides.setTargetPosition(pos);
    }

    /**
     * Method for moving the slides to their initial position.
     */
    public void init() {
        run(State.INIT);
    }

    public void cycleReady() {
        run(State.CYCLE_READY);}

    public void gigaHigh() {
        run(State.GIGA_HIGH);}

    public void autoHigh(){
        run(State.AUTO_HIGH);
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
    public void readyAuto() {
        run(State.AUTO_READY);
    }

    /**
     * Method for moving the slides to the fifth cone position.
     */
    public void fifthAutoCone() {
        run(State.CONE_5);}

    /**
     * Method for lowering slides from above cone stack
     * precondition: slides are at stationary, previous target position
     */
    public void lower(){
        // TODO: check current state, lower relative to current state:
        //  make enums for each cone stack height
        targetPos = State.LOW_AND_INTAKE;

        leftSlides.setTargetPosition(getCurrentPosition()-350);
        rightSlides.setTargetPosition(getCurrentPosition()-350);
    }

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
        double power = Range.clip(MIN_POWER + (Math.abs(targetPos.pos - getCurrentPosition()) * proportional),
                MIN_POWER, 1.0);

        // going down 0.8 power if substantial
        // TODO, fix loop to activate only when moving to different state
        if(targetPos.pos - getCurrentPosition() < 0 && Math.abs(targetPos.pos - getCurrentPosition()) > 200){
            setSlidesPower(0.8);
        }
        else {
            setSlidesPower(power);
        }
    }

    /**
     *
     * @param power
     */
    public void setSlidesPower(double power){
        leftSlides.setPower(power);
        rightSlides.setPower(power);
    }
}
