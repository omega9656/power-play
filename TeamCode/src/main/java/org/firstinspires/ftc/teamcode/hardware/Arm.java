package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo leftServo;
    public Servo rightServo;

    public ServoProfiler leftServoProfile;
    public ServoProfiler rightServoProfile;

    public enum Position {
        INIT(0.45), // was .5
        INTAKE(0),
        DEPOSIT(0.7); // was .75

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

        leftServoProfile.setConstraints(1.5, 1.5, 1.5);
        rightServoProfile.setConstraints(1.5, 1.5, 1.5);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        armPosition = Position.INIT;

        init();
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
     * Sets the position of the left and right servos to the "DEPOSIT" position.
     */
    public void deposit() {
        setArmPosition(Position.DEPOSIT);
    }

    /**
     * Updates the positions of the left and right servos.
     */
    public void update() {
        leftServoProfile.update();
        rightServoProfile.update(leftServoProfile);
    }
}
