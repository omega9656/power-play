package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DcMotorProfiler {
    private final DcMotorEx motor;
    private double maxVel, maxAccel, targetPosition, targetTolerance;
    private final ElapsedTime deltaTime;
    public double outputVelocity;

    // https://www.gobilda.com/content/spec_sheets/5204-8002-0014_spec_sheet.pdf
    public final double TICKS_PER_REVOLUTION = 384.5;  // encoder countable events per revolution (output shaft)
    public final int MAX_RPM = 435;  // *no-load* speed @ 12VDC
    public final double MAX_TICKS_PER_SEC = MAX_RPM * TICKS_PER_REVOLUTION / 60;  // 2787 ticks/sec
    public final double MAX_RADIANS_PER_SEC = ticksToRadians(MAX_TICKS_PER_SEC); // 45.55 rad/sec

    public static class Constraints {

        // max velocity is proportion of max rad/sec
        public double maxVelocity, maxAcceleration;

        public Constraints(double vel, double accel) {
            maxVelocity = vel;
            maxAcceleration = accel;
        }

        public double getMaxVelocity() {
            return maxVelocity;
        }

        public double getMaxAcceleration() {
            return maxAcceleration;
        }
    }


    /**
     * Constructor for DcMotorProfiler class.
     * @param m DcMotorEx object for the motor to be controlled
     */
    public DcMotorProfiler(DcMotorEx m) {
        motor = m;
        // tolerance seems high but this number is in ticks
        setTargetTolerance(2);
        deltaTime = new ElapsedTime();
    }

    /**
     * Set the maximum velocity and acceleration of the motor in terms of proportions of the maximum speed.
     * @param vel Proportion of maximum velocity
     * @param accel Proportion of maximum acceleration
     */
    public void setConstraints(double vel, double accel) {
        maxVel = vel * MAX_RADIANS_PER_SEC;
        maxAccel = accel;
    }

    /**
     * Set the maximum velocity and acceleration of the motor using a Constraints object.
     * @param c Constraints object containing the maximum velocity and acceleration
     */
    public void setConstraints(Constraints c) {
        setConstraints(c.maxVelocity, c.maxAcceleration);
    }

    /**
     * Set the target position for the motor to reach.
     * @param target target position in encoder ticks
     */
    public void setTargetPosition(int target) {
        targetPosition = target;
        motor.setTargetPosition(target);
        deltaTime.reset();
    }

    /**
     * Set the tolerance for the target position.
     * @param tolerance target position tolerance in encoder ticks
     */
    public void setTargetTolerance(double tolerance) {
        targetTolerance = tolerance;
    }


    /**
     * Update the motor's velocity based on the target position and the set constraints.
     */
    public void update() {
        double velocity = motor.getVelocity(AngleUnit.RADIANS);
        // get the change in time from the previous change
        // in velocity
        double deltaSec = deltaTime.seconds();

        double directionMultiplier = 1;

        // in radians
        double positionError = ticksToRadians(targetPosition - getCurrentPosition());

        if (positionError < 0){
            directionMultiplier = -1;
        }

        if (maxVel > Math.abs(velocity)) {
            outputVelocity = velocity + (directionMultiplier * maxAccel * deltaSec);
        } else {
            outputVelocity = maxVel;
        }

        if (Math.abs(positionError) <= (Math.pow(outputVelocity, 2) / (2 * maxAccel))) {
            outputVelocity = velocity - (directionMultiplier * maxAccel * deltaSec);
        }

        if (isAtTarget()) {
            outputVelocity = 0;
            motor.setVelocity(outputVelocity);
            return;
        }

        motor.setVelocity(outputVelocity, AngleUnit.RADIANS);
        deltaTime.reset();
    }

    /**
     * Update the motor's velocity based on the target position and the set constraints, copying the output velocity from another DcMotorProfiler.
     * @param copy DcMotorProfiler to copy output velocity from
     */
    public void update(DcMotorProfiler copy){
        outputVelocity = copy.outputVelocity;
        motor.setVelocity(outputVelocity, AngleUnit.RADIANS);
        if(isAtTarget()) return;
    }

    /**
     Calculates the distance traveled in a motion profile given the elapsed time, total distance, and maximum velocity.
     @param curr_dt The current time elapsed in the motion.
     @param distance The total distance to travel.
     @param maxVelocity The maximum velocity allowed.
     @return The distance traveled in the motion profile.
     */
    public double motionProfile(double curr_dt, double distance, double maxVelocity){
        double acc_dt = maxVelocity / maxAccel;

        double halfway_distance = distance/2;
        double accel_distance = 0.5 * maxAccel * Math.pow(acc_dt, 2);

        if(accel_distance > halfway_distance){
            acc_dt = Math.sqrt(halfway_distance / (0.5 * maxAccel));
        }
        accel_distance = 0.5 * maxAccel * Math.pow(acc_dt, 2);

        maxVelocity = maxAccel * acc_dt;

        double cruiseDistance = distance - (2 * accel_distance);
        double cruise_dt = cruiseDistance / maxVelocity;
        double deaccel_time = acc_dt + cruise_dt;

        double total_dt = acc_dt + cruise_dt + deaccel_time;
        if(curr_dt > total_dt) return distance;

        if(curr_dt < acc_dt) return 0.5 * maxAccel * Math.pow(curr_dt, 2);
        else if(curr_dt < deaccel_time) {
            accel_distance = 0.5 * maxAccel * Math.pow(acc_dt, 2);
            return accel_distance + (maxVelocity * (curr_dt-acc_dt));
        }

        accel_distance = 0.5 * maxAccel * Math.pow(acc_dt, 2);
        cruiseDistance = maxVelocity * cruise_dt;
        deaccel_time = curr_dt - deaccel_time;

        // d = d0 + v0*t + 0.5*a*t^2
        return (accel_distance + cruiseDistance) + maxVelocity * deaccel_time - 0.5 * maxAccel * Math.pow(deaccel_time, 2);
    }

    /**
     * Returns true if the motor is within the target tolerance of the target position, false otherwise.
     * @return Boolean indicating if the motor is at target
     */
    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) <= targetTolerance;
    }

    /**
     * Returns the current position of the motor in encoder ticks.
     * @return Current position in encoder ticks
     */
    public double getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /**
     * Returns the DcMotorEx object being controlled by the DcMotorProfiler.
     * @return DcMotorEx object
     */
    public DcMotorEx getMotor() {
        return motor;
    }

    /**
     * Returns the maximum velocity of the motor in radians per second.
     * @return maximum velocity in radians per second
     */
    public double getMaxVel() {
        return maxVel;
    }

    /**
     * Returns the maximum acceleration of the motor in radians per second squared.
     * @return maximum acceleration in radians per second squared
     */
    public double getMaxAccel() {
        return maxAccel;
    }

    /**
     * Returns the target position of the motor in encoder ticks.
     * @return target position in encoder ticks
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns the target tolerance of the motor in encoder ticks.
     * @return target tolerance in encoder ticks
     */
    public double getTargetTolerance() {
        return targetTolerance;
    }

    /**
     * Converts encoder ticks to radians.
     * @param ticks Encoder ticks
     * @return Radians
     */
    public double ticksToRadians(double ticks) {return 2 * Math.PI * ticks / TICKS_PER_REVOLUTION;}
}