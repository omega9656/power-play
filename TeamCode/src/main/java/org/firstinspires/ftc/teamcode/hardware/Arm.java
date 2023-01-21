package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    public Servo leftServo;
    public Servo rightServo;

    public ServoProfiler leftServoProfile;
    public ServoProfiler rightServoProfile;

    public enum Position {
        INIT(0.36), // was 0.35, .45
        EXTENDO_DEPOSIT(0.28),
        INTAKE(0),
        DEPOSIT(0.52), // was .6
        AUTO_DEPOSIT(0.6),
        GIGA_EXTENDO(1);

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

        leftServoProfile.setConstraints(1.75, 1, 2);
        rightServoProfile.setConstraints(1.75, 1, 2);

        leftServo.setPosition(0);
        rightServo.setPosition(0);

        armPosition = Position.INIT;

        init();
    }

    public void setConstraints(double vel, double accel, double prop){
        leftServoProfile.setConstraints(vel, accel, prop);
        rightServoProfile.setConstraints(vel, accel, prop);
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

    public void giga() {setArmPosition(Position.GIGA_EXTENDO);}

    public void extendoDeposit() {
        setArmPosition(Position.EXTENDO_DEPOSIT);
    }

    public void autoDeposit() {setArmPosition(Position.AUTO_DEPOSIT);}

    public void update() {
        leftServoProfile.update();
        rightServoProfile.update(leftServoProfile);
    }


}
