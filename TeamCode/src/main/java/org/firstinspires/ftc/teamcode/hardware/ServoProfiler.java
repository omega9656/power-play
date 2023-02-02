package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class ServoProfiler {
    private final Servo servo;
    private double maxVel, maxAccel, targetPosition, servoRange, targetTolerance;
    private final ElapsedTime deltaTime;
    private double delta;
    private double proportion;

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

    /**
     * Constructor for ServoProfiler class.
     *
     * @param s Servo object to be profiled.
     */
    public ServoProfiler(Servo s) {
        servo = s;
        setServoRange(1);
        setTargetTolerance(0.01);
        deltaTime = new ElapsedTime();
        delta = 0;
    }

    /**
     * Sets the maximum velocity, maximum acceleration, and proportion constraints.
     *
     * @param vel Maximum velocity constraint.
     * @param accel Maximum acceleration constraint.
     * @param prop Proportion constraint.
     * @return ServoProfiler object with updated constraints.
     */
    public ServoProfiler setConstraints(double vel, double accel, double prop) {
        maxVel = vel;
        maxAccel = accel;
        proportion = prop;
        return this;
    }

    /**
     * Sets the maximum velocity, maximum acceleration, and proportion constraints using a Constraints object.
     *
     * @param c Constraints object containing the maximum velocity, maximum acceleration, and proportion constraints.
     * @return ServoProfiler object with updated constraints.
     */
    public ServoProfiler setConstraints(Constraints c) {
        return setConstraints(c.maxVelocity, c.maxAcceleration, c.proportion);
    }

    /**
     * Sets the range of the servo in rotations.
     *
     * @param rangeInRotations Range of the servo in rotations.
     * @return ServoProfiler object with updated servo range.
     */
    public ServoProfiler setServoRange(double rangeInRotations) {
        servoRange = rangeInRotations;
        return this;
    }

    /**
     * Sets the target position of the servo and resets the delta time.
     *
     * @param target Target position of the servo.
     * @return ServoProfiler object with updated target position.
     */
    public ServoProfiler setTargetPosition(double target) {
        targetPosition = target;
        deltaTime.reset();
        return this;
    }

    /**
     * Translates the target position of the servo by a certain amount.
     *
     * @param translation Amount to translate the target position.
     * @return ServoProfiler object with updated target position.
     */
    public ServoProfiler translateTargetPosition(double translation) {
        return setTargetPosition(Range.clip(0, targetPosition + translation, 1));
    }

    /**
     * Sets the tolerance for determining if the servo has reached its target position.
     *
     * @param tolerance Tolerance for determining if the servo has reached its target position.
     * @return ServoProfiler object with updated target tolerance.
     */
    public ServoProfiler setTargetTolerance(double tolerance) {
        targetTolerance = tolerance;
        return this;
    }

    /**
     * Updates the position of the servo based on the set constraints and target position.
     *
     * @return ServoProfiler object with updated servo position.
     */
    public ServoProfiler update() {

        // set the past change in servo position
        double pastDelta = delta;
        // get the change in time from the previous change
        // in position, then reset the timer instantly
        double deltaSec = deltaTime.seconds();
        deltaTime.reset();

        // generate the new change in servo pos.
        // range.clip makes the change fit the max constraints
        // the min and max make sure both constraints are hit
        // the deltasec makes it independent of looptime
        delta = Range.clip(
                deltaSec * servoRange * proportion *
                        (getTargetPosition() - getCurrentPosition()),
                //
                Math.max(pastDelta - maxAccel * deltaSec, -maxVel * deltaSec),
                Math.min(pastDelta + maxAccel * deltaSec, maxVel * deltaSec));
        servo.setPosition(getCurrentPosition() + delta / servoRange);

        // if the servo is at its target position, stop moving
        if (isAtTarget()) return this;
        return this;
    }

    /**
     * Copies the motion profile of another ServoProfiler object.
     *
     * @param copy ServoProfiler object to copy motion profile from.
     * @return ServoProfiler object with updated servo position.
     */
    public ServoProfiler update(ServoProfiler copy){

        servo.setPosition(getCurrentPosition() + copy.delta / servoRange);
        if(isAtTarget()) return this;
        return this;
    }

    /**
     * Checks if the servo has reached its target position.
     *
     * @return true if the servo has reached its target position, false otherwise.
     */
    public boolean isAtTarget() {
        return Math.abs(getCurrentPosition() - getTargetPosition()) < targetTolerance;
    }

    /**
     * Gets the current position of the servo.
     *
     * @return Current position of the servo.
     */
    public double getCurrentPosition() {
        return servo.getPosition();
    }

    /**
     * Gets the Servo object being profiled.
     *
     * @return Servo object being profiled.
     */
    public Servo getServo() {
        return servo;
    }

    /**
     * Returns the maxVel field.
     * @return Maximum velocity constraint.
     */
    public double getMaxVel() {
        return maxVel;
    }

    /**
     * Returns the maxAccel field.
     * @return Maximum acceleration constraint.
     */
    public double getMaxAccel() {
        return maxAccel;
    }

    /**
     * Returns the targetPosition field.
     * @return Target position of the servo.
     */
    public double getTargetPosition() {
        return targetPosition;
    }

    /**
     * Returns the targetTolerance field.
     * @return Tolerance for determining if the servo has reached its target position.
     */
    public double getTargetTolerance() {
        return targetTolerance;
    }

}
