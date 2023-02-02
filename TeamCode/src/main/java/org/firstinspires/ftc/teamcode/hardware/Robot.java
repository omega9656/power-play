package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public DeviceManager deviceManager;

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;
    public Slides slides;

    /**
     * Robot class is used to manage all the hardware objects for the robot.
     * @param hardwareMap HardwareMap object for the robot
     */
    public Robot(HardwareMap hardwareMap) {
        deviceManager = new DeviceManager(hardwareMap);
    }

    /**
     * Initializes the DeviceManager and creates the Drivetrain, Arm, Intake and Slides objects.
     * @param runningAuto boolean indicating if the robot is running in autonomous mode
     * @param afterAuto boolean indicating if the robot is running after autonomous mode
     */
    public void init(boolean runningAuto, boolean afterAuto) {
        deviceManager.init(runningAuto);

        if (!runningAuto) {
            drivetrain = new Drivetrain(deviceManager);
        }

        arm = new Arm(deviceManager);
        intake = new Intake(deviceManager);
        slides = new Slides(deviceManager, afterAuto);
    }
}
