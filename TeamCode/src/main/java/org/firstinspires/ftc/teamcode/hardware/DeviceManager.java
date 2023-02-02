package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DeviceManager {
    public HardwareMap hardwareMap;

    // both motors are connected
    // to each other with a shaft
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    // motor controlling intake
    public DcMotorEx intake;

    // drivetrain hardware
    public DcMotorEx backRight;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx frontLeft;

    // arm hardware
    public Servo leftServo;
    public Servo rightServo;

    /**
     * DeviceManager class is used to manage the hardware devices on the robot.
     * @param hardwareMap HardwareMap object for the robot
     */
    public DeviceManager(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Initializes the hardware devices for teleop.
     */
    public void initTeleop() {
        init(false);
    }

    /**
     * Initializes the hardware devices for autonomous.
     */
    public void initAuto() {
        init(true);
    }

    /**
     * Initializes the hardware devices.
     * @param autoRunning boolean indicating if the robot is in autonomous mode
     */
    public void init(boolean autoRunning) {
        if (!autoRunning) {
            backRight = hardwareMap.get(DcMotorEx.class, "back_right");
            frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
            backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
            frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        }

        leftSlides = hardwareMap.get(DcMotorEx.class, "left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class, "right_slides");

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");
    }
}
