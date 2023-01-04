package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.RobotOld;
import org.firstinspires.ftc.teamcode.hardware.ServoProfiler;

@TeleOp
public class ServoTest extends OpMode {
    Robot robot;

    enum DriveMode {
        SQUARED, CUBED, NORMAL
    }

    ElapsedTime time;

    boolean intake;

    @Override
    public void init() {
        intake = false;
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        robot = new Robot(hardwareMap);
        robot.init(false, false);

        robot.arm.setArmPosition(Arm.Position.INTAKE);

//        robot.leftS.setTargetPosition(0.5);
//        robot.rightS.setTargetPosition(0.5);


//        telemetry.addData("left servo curr", robot.leftS.getCurrentPosition());
//        telemetry.addData("left servo targ", robot.leftS.getTargetPosition());
//        telemetry.addData("servo direction", robot.getDirection());
    }

    @Override
    public void init_loop() {

        robot.arm.leftServoProfile.update();
        robot.arm.rightServoProfile.update();

        telemetry.addData("left servo curr", robot.arm.leftServo.getPosition());
        telemetry.addData("right servo curr", robot.arm.rightServo.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.arm.leftServoProfile.update();
        robot.arm.rightServoProfile.update();

        moveServos();

        telemetry.addData("left servo curr", robot.arm.leftServo.getPosition());
        telemetry.addData("right servo curr", robot.arm.rightServo.getPosition());
        telemetry.update();
    }

    public void moveServos(){
        // intake position, no motion profile
        if(gamepad2.b){
            robot.arm.setArmPosition(Arm.Position.INTAKE);
        }

        // hold above intake
        if(gamepad2.a){
            robot.arm.setArmPosition(Arm.Position.INIT);
        }
    }
}
