package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp
public class DrivetrainTest extends OpMode {

    public DcMotorEx backRight;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx frontLeft;

    public Encoder leftEncoder;
    public Encoder rightEncoder;
    public Encoder frontEncoder;

    public double leftPos;
    public double rightPos;
    public double frontPos;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        backRight = hardwareMap.get(DcMotorEx.class, "back_right");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "front_left"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back_right"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "middle_encoder"));

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPos = leftEncoder.getCurrentPosition();
        rightPos = leftEncoder.getCurrentPosition();
        frontPos = leftEncoder.getCurrentPosition();
    }

    @Override
    public void loop() {

        telemetry.addData("left", leftEncoder.getCurrentPosition() - leftPos);
        telemetry.addData("right", rightEncoder.getCurrentPosition() - rightPos);
        telemetry.addData("front", frontEncoder.getCurrentPosition() - frontPos);
        telemetry.update();
    }
}
