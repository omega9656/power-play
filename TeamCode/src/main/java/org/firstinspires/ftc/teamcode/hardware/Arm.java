package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo leftServo;
    public Servo rightServo;

    public ServoProfiler leftServoProfile;
    public ServoProfiler rightServoProfile;

    // TODO make state machines if constants vary across opmode
    public final static double VEL = 1.75;
    public final static double ACCEL = 1;
    public final static double PROP = 2;

    public enum Position {
        INIT(0.36),
        EXTENDO_DEPOSIT(0.28),
        INTAKE(0),
        DEPOSIT(0.55),
        AUTO_DEPOSIT(0.6),
        GIGA_EXTENDO(0.95);

        // this is a value from 0 to 1
        public double pos;

        Position (double servoPos) {
            pos = servoPos;
        }
    }

    public Position armPosition;

    /**
     * Constructor for the Arm class. Assigns the left and right servos, sets the direction
     * of the servos, and sets the initial position of the servos.
     *
     * @param deviceManager The DeviceManager object used to get the left and right servos.
     */
    public Arm(DeviceManager deviceManager) {
        leftServo = deviceManager.leftServo;
        rightServo = deviceManager.rightServo;

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
        leftServoProfile = new ServoProfiler(leftServo);
        rightServoProfile = new ServoProfiler(rightServo);

        leftServoProfile.setConstraints(VEL, ACCEL, PROP);
        rightServoProfile.setConstraints(VEL, ACCEL, PROP);

        // TODO: uncomment if smt wrong w/ servos
        leftServo.setPosition(leftServo.getPosition());
        rightServo.setPosition(rightServo.getPosition());

        armPosition = Position.INIT;

        init();
    }

    public void setConstraints(double vel, double accel, double prop){
        leftServoProfile.setConstraints(vel, accel, prop);
        rightServoProfile.setConstraints(vel, accel, prop);
    }

    /**
     * Sets the position of the left and right servos to the specified position.
     *
     * @param pos The position to set the servos to.
     */
    public void setArmPosition(Position pos) {
        armPosition = pos;
        leftServoProfile.setTargetPosition(pos.pos);
        rightServoProfile.setTargetPosition(pos.pos);
    }

    /**
     * sets both arm servo positions to given, used for value testing purposes before transferring to enums
     * @param pos value between 0 and 1
     */
    public void setArmPosition(double pos){
        leftServoProfile.setTargetPosition(pos);
        rightServoProfile.setTargetPosition(pos);
    }

    /**
     * Sets the position of the left and right servos to the "INTAKE" position.
     */
    public void intake() {
        setArmPosition(Position.INTAKE);
    }

    /**
     * Sets the position of the left and right servos to the "INIT" position.
     */
    public void init() {
        setArmPosition(Position.INIT);
    }

    /**
     * Sets the position of the left and right servos to the "DEPOSIT" position
     * -- for outtaking cones from the back of the robot
     */
    public void deposit() {
        setArmPosition(Position.DEPOSIT);
    }

    /**
     * Sets the position of the left and right servos to the "GIGA EXTENDO" intake position
     * -- for cycling on high from terminal
     */
    public void extendoIntake() {setArmPosition(Position.GIGA_EXTENDO);}

    /**
     * Sets the position of the left and right servos to the "EXTENDO DEPOSIT" outtake position
     * -- for cycling on high from terminal
     */
    public void extendoDeposit() {
        setArmPosition(Position.EXTENDO_DEPOSIT);
    }

    /**
     * gets current servo position
     * @return current position of the left servo (should be synced with right servo)
     */
    public double getCurrentPosition() {
        return leftServoProfile.getCurrentPosition();
    }

    /**
     * gets target servo position
     * @return target position of the left servo (should be the same as the right servo)
     */
    public double getTargetPosition() {
        return leftServoProfile.getTargetPosition();
    }

    /**
     * Updates the positions of the left and right servos.
     */
    public void update() {
        leftServoProfile.update();
        rightServoProfile.update(leftServoProfile);
    }




}
