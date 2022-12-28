package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {
    public DeviceManager deviceManager;

    public Drivetrain drivetrain;
    public Arm arm;
    public Intake intake;

    public Robot(HardwareMap hardwareMap) {
        deviceManager = new DeviceManager(hardwareMap);
    }

    public void init(boolean runningAuto) {
        deviceManager.init(runningAuto);

        if (!runningAuto) {
            drivetrain = new Drivetrain(deviceManager);
        }

        arm = new Arm(deviceManager);
        intake = new Intake(deviceManager);
    }
}
