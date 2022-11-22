package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {
    public DcMotorEx leftSlides;
    public DcMotorEx rightSlides;

    public DcMotorEx intake;

    public DcMotorEx backRight;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx frontLeft;

    public Servo leftServo;
    public Servo rightServo;

    public enum ServoDirection {
        INTAKE, OUTTAKE
    }

    ServoDirection s;

    public ServoDirection getDirection() {return s;}

    public void init(HardwareMap hardwareMap, boolean autoRunning){
        s = ServoDirection.INTAKE;
        if (!autoRunning) {
            backRight = hardwareMap.get(DcMotorEx.class, "back_right");
            frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
            backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
            frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");

            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            //backRight.setDirection(DcMotorSimple.Direction.REVERSE);
            //frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        leftSlides = hardwareMap.get(DcMotorEx.class, "left_slides");
        rightSlides = hardwareMap.get(DcMotorEx.class, "right_slides");

        rightSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        leftSlides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        // sets default position
        rightSlides.setTargetPosition(0);
        leftSlides.setTargetPosition(0);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE); // when power is 0, BRAKE

        leftServo = hardwareMap.get(Servo.class, "left_servo");
        rightServo = hardwareMap.get(Servo.class, "right_servo");

        rightServo.setDirection(Servo.Direction.FORWARD);
        leftServo.setDirection(Servo.Direction.REVERSE);

        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void flipDirection(){
        if(s == ServoDirection.OUTTAKE){
            s = ServoDirection.INTAKE;
            rightServo.setDirection(Servo.Direction.FORWARD);
            leftServo.setDirection(Servo.Direction.REVERSE);
            return;
        }
        s = ServoDirection.OUTTAKE;
        rightServo.setDirection(Servo.Direction.REVERSE);
        leftServo.setDirection(Servo.Direction.FORWARD);
    }

    public void setServoPos(double pos){
        rightServo.setPosition(pos);
        leftServo.setPosition(pos);
    }
}
