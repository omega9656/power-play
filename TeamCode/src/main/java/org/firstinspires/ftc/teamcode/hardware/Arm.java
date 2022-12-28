package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo leftServo;
    public Servo rightServo;

    public ServoProfiler leftServoProfile;
    public ServoProfiler rightServoProfile;

    public enum Position {
        INIT(0.5),
        INTAKE(0),
        DEPOSIT(0.75);

        // this is a value from 0 to 1
        double pos;

        Position (double servoPos) {
            pos = servoPos;
        }
    }

    public Position armPosition;

    public void setArmPosition(Position pos) {
        armPosition = pos;
    }

    public void down() {
        setArmPosition(Position.INTAKE);
    }

    public void up() {
        setArmPosition(Position.DEPOSIT);
    }

    public Arm(DeviceManager deviceManager) {
        leftServo = deviceManager.leftServo;
        rightServo = deviceManager.rightServo;

        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);

        leftServoProfile = new ServoProfiler(leftServo);
        rightServoProfile = new ServoProfiler(rightServo);

        leftServoProfile.setConstraints(1.2, 1.2, 1);
        rightServoProfile.setConstraints(1.2, 1.2, 1);
    }


}
