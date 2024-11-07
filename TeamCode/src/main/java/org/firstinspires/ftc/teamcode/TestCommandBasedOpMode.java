package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;


@TeleOp(name="Test CommandBase OpMode")

public class TestCommandBasedOpMode extends CommandOpMode {

    private GamepadEx driver, operator;
    private MotorEx FrontL, FrontR, BackL, BackR;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;
    private Button slowButton;
    @Override
    public void initialize(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        FrontL = hardwareMap.get(MotorEx.class, "fl(eet footwork)");
        FrontR = hardwareMap.get(MotorEx.class, "fr(ank)");
        BackL = hardwareMap.get(MotorEx.class, "bl(itzcrank)");
        BackR = hardwareMap.get(MotorEx.class, "br(iar)");

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);
        driveCommand = new DriveCommand(driveSubsystem, driver.getLeftX(), driver.getLeftY(),
                                        driver.getRightX(), driver.isDown(GamepadKeys.Button.X));

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);
    }
}
