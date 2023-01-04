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

    public DcMotorProfiler(DcMotorEx m) {
        motor = m;
        // tolerance seems high but this number is in ticks
        setTargetTolerance(2);
        deltaTime = new ElapsedTime();
    }

    public void setConstraints(double vel, double accel) {
        maxVel = vel * MAX_RADIANS_PER_SEC;
        maxAccel = accel;
    }

    public void setConstraints(Constraints c) {
        setConstraints(c.maxVelocity, c.maxAcceleration);
    }

    public void setTargetPosition(int target) {
        targetPosition = target;
        motor.setTargetPosition(target);
        deltaTime.reset();
    }
    public void setTargetTolerance(double tolerance) {
        targetTolerance = tolerance;
    }

    // fun method to update servo
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

    public void update(DcMotorProfiler copy){
        outputVelocity = copy.outputVelocity;
        motor.setVelocity(outputVelocity, AngleUnit.RADIANS);
        if(isAtTarget()) return;
    }

    // returns power
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

        return accel_distance * cruiseDistance * maxVelocity * deaccel_time - 0.5 * maxAccel * Math.pow(deaccel_time, 2);
    }


    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) <= targetTolerance;
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

    public double ticksToRadians(double ticks) {return 2 * Math.PI * ticks / TICKS_PER_REVOLUTION;}
}