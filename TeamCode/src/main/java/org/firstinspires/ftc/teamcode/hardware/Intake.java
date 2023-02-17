package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private DcMotorEx motor;
    public Mode state;

    // https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-5-2-1-ratio-1150-rpm-3-3-5v-encoder/
    // see stall current at 12VDC
    private final double STALL_CURRENT = 9.2;

    public enum Mode {
        IN(-0.6),   // intake the cone
        HOLD(-0.3), // hold cone in place when moving
        OUT(0.6), // deposit the cone
        STOP(0);

        public double power;

        Mode(double power) {
            this.power = power;
        }
    }

    /**
     * Intake class is used to manage the intake motor on the robot.
     * @param deviceManager DeviceManager object for the robot
     */
    public Intake(DeviceManager deviceManager) {
        motor = deviceManager.intake;

        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // when power is 0, BRAKE

        state = Mode.STOP;
        stop();
    }

    /**
     * Sets the power of the motor to the corresponding value and updates the state variable.
     * @param mode Mode enumeration value representing the desired state of the intake.
     */
    private void run(Mode mode) {
        motor.setPower(mode.power);
        state = mode;
    }

    /**
     * Intake cone
     */
    public void in() {
        run(Mode.IN);
    }

    /**
     * Hold cone in place when moving
     */
    public void hold() {
        run(Mode.HOLD);
    }

    /**
     * Deposit the cone
     */
    public void out() {
        run(Mode.OUT);
    }

    /**
     * Stop the intake motor
     */
    public void stop() {run(Mode.STOP);}

    /**
     * sets power of the motor to value passed, used for testing purposes
     * @param power Value representing proportion of power being sent to motor [0,1]
     */
    public void setPower(double power){
        motor.setPower(power);
    }

    /**
     * returns intake motor power
     * @return motor power [0,1]
     */
    public double getPower(){
        return motor.getPower();
    }

    /**
     * checks if motor hit stall current
     * @return true if current is greater than stall current,
     */
    public boolean isStalled(){
        return getAmps() >= STALL_CURRENT;
    }

    /**
     * returns motor current
     * @return current in amps
     */
    public double getAmps(){
        return motor.getCurrent(CurrentUnit.AMPS);
    }
}
