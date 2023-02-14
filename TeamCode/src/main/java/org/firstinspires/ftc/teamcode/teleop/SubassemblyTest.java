package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class SubassemblyTest extends OpMode {
    Robot robot;

    // arm
    public static boolean testArm = true;
    public static double intake = 0;
    public static double init = 0;
    public static double deposit = 0;
    public static double intakeDeposit = 0;

    // slides
    public static boolean testSlides = false;
    public static int high = 0;
    public static int mid = 0;
    public static int low = 0;
    public static int slidesIntake = 0;

    public static int cone5 = 0;
    public static int cone4 = 0;
    public static int cone3 = 0;
    public static int cone2 = 0;

    // intake
    public static boolean testIntake = false;
    public boolean stalled = false;
    public double inPower = 0;
    public double holdPower = 0;
    public double outPower = 0;


    public double stallCurrent = 9.2; // amps


    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    public void moveArm(){
        // zero position, intake
        if(gamepad2.b){
            robot.arm.setArmPosition(intake);
        }
        // init position
        if(gamepad2.a){
            robot.arm.setArmPosition(init);
        }
        // normal deposit position
        if(gamepad2.x){
            robot.arm.setArmPosition(deposit);
        }
    }

    public void intake(){
        //hold
        if(gamepad2.left_bumper || robot.intake.isStalled()){
            robot.intake.setPower(holdPower);
        }
        //outtake
        else if(gamepad2.left_trigger > 0.3){
            robot.intake.setPower(outPower);
        }
        //intake
        else if(gamepad2.right_trigger > 0.3){
            robot.intake.setPower(inPower);
        }
        else {
            robot.intake.setPower(0);
        }
    }

    @Override
    public void loop() {
        if(testArm) {
            moveArm();

            telemetry.addLine("Gamepad 2: ");
            telemetry.addLine("         B: intake ");
            telemetry.addLine("         A: init ");
            telemetry.addLine("         X: deposit ");

            telemetry.addData("current arm pos: ", robot.arm.getCurrentPosition());
            telemetry.addData("target arm pos: ", robot.arm.getTargetPosition());
        }

        if(testIntake){
            intake();

            telemetry.addLine("Gamepad 2: ");
            telemetry.addLine("         Right trigger: intake ");
            telemetry.addLine("         Left trigger: outtake ");
            telemetry.addLine("         Left bumper: hold ");

            telemetry.addData("Is the intake motor stalled? ", stalled);
            telemetry.addData("current intake power: ", robot.intake.getPower());
        }


        telemetry.update();
    }
}
