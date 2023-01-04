package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo leftServo;
    public Servo rightServo;

    public ServoProfiler leftServoProfile;
    public ServoProfiler rightServoProfile;

    public enum Position {
        INIT(0.6), // was .5
        INTAKE(0),
        DEPOSIT(0.85); // was .75

        // this is a value from 0 to 1
        public double pos;

        Position (double servoPos) {
            pos = servoPos;
        }
    }

    public Position armPosition;

    public Arm(DeviceManager deviceManager) {
        leftServo = deviceManager.leftServo;
        rightServo = deviceManager.rightServo;

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);

        leftServoProfile = new ServoProfiler(leftServo);
        rightServoProfile = new ServoProfiler(rightServo);

        leftServoProfile.setConstraints(1.2, 1.2, 1);
        rightServoProfile.setConstraints(1.2, 1.2, 1);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        armPosition = Position.INIT;

        init();
    }

    public void setArmPosition(Position pos) {
        armPosition = pos;
        leftServoProfile.setTargetPosition(pos.pos);
        rightServoProfile.setTargetPosition(pos.pos);
    }

    public void intake() {
        setArmPosition(Position.INTAKE);
    }

    public void init() {
        setArmPosition(Position.INIT);
    }

    public void deposit() {
        setArmPosition(Position.DEPOSIT);
    }


}
