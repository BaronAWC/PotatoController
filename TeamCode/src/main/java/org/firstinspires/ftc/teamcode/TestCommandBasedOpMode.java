package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;


@TeleOp(name="Test CommandBase OpMode")

public class TestCommandBasedOpMode extends CommandOpMode {

    private GamepadEx driver, operator;
    private DcMotorEx FrontL, FrontR, BackL, BackR;
    private DcMotorEx arm, pivot;
    private CRServo intake;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private ArmSubsystem armSubsystem;
    private ArmExtendCommand armExtendCommand;
    private ArmRetractCommand armRetractCommand;
    private ArmStopCommand armStopCommand;

    private PivotSubsystem pivotSubsystem;
    private PivotRaiseCommand pivotRaiseCommand;
    private PivotLowerCommand pivotLowerCommand;
    private PivotStopCommand pivotStopCommand;

    private IntakeSubsystem intakeSubsystem;
    private IntakeForwardCommand intakeForwardCommand;
    private IntakeBackwardCommand intakeBackwardCommand;
    private IntakeStopCommand intakeStopCommand;

    private Button extendButton, retractButton, pivotUpButton, pivotDownButton, intakeForwardButton, intakeBackwardButton;
    @Override
    public void initialize(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        FrontL = hardwareMap.get(DcMotorEx.class, "fl(eet footwork)");
        FrontR = hardwareMap.get(DcMotorEx.class, "fr(ank)");
        BackL = hardwareMap.get(DcMotorEx.class, "bl(itzcrank)");
        BackR = hardwareMap.get(DcMotorEx.class, "br(iar)");

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu, telemetry);
        driveCommand = new DriveCommand(driveSubsystem, driver.getLeftX(), driver.getLeftY(),
                                        driver.getRightX(), driver.isDown(GamepadKeys.Button.X));

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        intake = hardwareMap.get(CRServo.class, "spinnything");

        armSubsystem = new ArmSubsystem(arm);
        armExtendCommand = new ArmExtendCommand(armSubsystem);
        armRetractCommand = new ArmRetractCommand(armSubsystem);
        armStopCommand = new ArmStopCommand(armSubsystem);

        pivotSubsystem = new PivotSubsystem(pivot);
        pivotRaiseCommand = new PivotRaiseCommand(pivotSubsystem);
        pivotLowerCommand = new PivotLowerCommand(pivotSubsystem);
        pivotStopCommand = new PivotStopCommand(pivotSubsystem);

        intakeSubsystem = new IntakeSubsystem(intake);
        intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
        intakeBackwardCommand = new IntakeBackwardCommand(intakeSubsystem);
        intakeStopCommand = new IntakeStopCommand(intakeSubsystem);

        extendButton = (new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)).whenPressed(armExtendCommand);
        retractButton = (new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)).whenPressed(armRetractCommand);
        extendButton.whenReleased(armStopCommand);
        retractButton.whenReleased(armStopCommand);

        pivotUpButton = (new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)).whenPressed(pivotRaiseCommand);
        pivotDownButton = (new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)).whenPressed(pivotLowerCommand);
        pivotUpButton.whenReleased(pivotStopCommand);
        pivotDownButton.whenReleased(pivotStopCommand);

        intakeForwardButton = (new GamepadButton(operator, GamepadKeys.Button.B)).whenPressed(intakeForwardCommand);
        intakeBackwardButton = (new GamepadButton(operator, GamepadKeys.Button.X)).whenPressed(intakeBackwardCommand);
        intakeForwardButton.whenReleased(intakeStopCommand);
        intakeBackwardButton.whenReleased(intakeStopCommand);
    }
}
