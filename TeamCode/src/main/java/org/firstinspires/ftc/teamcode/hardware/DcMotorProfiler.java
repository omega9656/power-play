package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DcMotorProfiler {
    private final DcMotorEx motor;
    private double maxVel, maxAccel, targetPosition, targetTolerance;
    private final ElapsedTime deltaTime;
    private double delta;

    public static class Constraints {

        public double maxVelocity, maxAcceleration, proportion;

        public Constraints(double vel, double accel, double prop) {
            maxVelocity = vel;
            maxAcceleration = accel;
            proportion = prop;
        }

        public double getMaxVelocity() {
            return maxVelocity;
        }

        public double getMaxAcceleration() {
            return maxAcceleration;
        }

        public double getProportion() {
            return proportion;
        }
    }

    public DcMotorProfiler(DcMotorEx m) {
        motor = m;
        setTargetTolerance(0.01);
        deltaTime = new ElapsedTime();
        delta = 0;
    }

    public void setConstraints(double vel, double accel) {
        maxVel = vel;
        maxAccel = accel;
//        proportion = prop;
    }

    public void setConstraints(Constraints c) {
        setConstraints(c.maxVelocity, c.maxAcceleration);
    }

    public void setTargetPosition(double target) {
        targetPosition = target;
        deltaTime.reset();
    }

//    public void translateTargetPosition(double translation) {
//        setTargetPosition(Range.clip(0, targetPosition + translation, 1));
//    }

    public void setTargetTolerance(double tolerance) {
        targetTolerance = tolerance;
    }

    // fun method to update servo
    public void update() {
        double velocity = motor.getVelocity();

        // set the past change in servo position
        double pastDelta = delta;
        // get the change in time from the previous change
        // in position, then reset the timer instantly
        double deltaSec = deltaTime.seconds();
        deltaTime.reset();

        double directionMultiplier = 1;

        double positionError = targetPosition - getCurrentPosition();

        if (positionError < 0)
            directionMultiplier = -1;

        double outputVelocity;
//        double outputAcceleration;

        if (maxVel > Math.abs(velocity)) {
            outputVelocity = velocity + directionMultiplier * maxAccel * (deltaSec - pastDelta);
//            outputAcceleration = maxAccel;
        } else {
            outputVelocity = maxVel;
//            outputAcceleration = 0;
        }

        if (positionError <= (Math.pow(outputVelocity, 2) / (2 * maxAccel))) {
            outputVelocity = velocity - directionMultiplier * maxAccel * (deltaSec - pastDelta);
//            outputAcceleration = -maxAccel;
        }

        if (isAtTarget())
            return;

        motor.setVelocity(outputVelocity);
    }


    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) < targetTolerance;
    }

    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    public double getMaxVel() {
        return maxVel;
    }

    public double getMaxAccel() {
        return maxAccel;
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getTargetTolerance() {
        return targetTolerance;
    }
}