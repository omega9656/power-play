package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    public DcMotorEx motor;
    public Mode state;

    public enum Mode {
        IN(0.6),   // intake the cone
        HOLD(0.1), // hold cone in place when moving
        OUT(-0.6), // deposit the cone
        STOP(0);

        public double power;

        Mode(double power) {
            this.power = power;
        }
    }

    public Intake(DeviceManager deviceManager) {
        motor = deviceManager.intake;

        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // when power is 0, BRAKE

        state = Mode.STOP;
        stop();
    }

    public void run(Mode mode) {
        motor.setPower(mode.power);
        state = mode;
    }

    public void in() {
        run(Mode.IN);
    }

    public void hold() {
        run(Mode.HOLD);
    }

    public void out() {
        run(Mode.OUT);
    }

    public void stop() {run(Mode.STOP);}
}
