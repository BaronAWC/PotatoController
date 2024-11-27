package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.qualcomm.robotcore.hardware.CRServo;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


@TeleOp(name="CommandBase OpMode")

public class CommandBasedOpMode extends CommandOpMode {

    private GamepadEx driver, operator;
    private DcMotorEx FrontL, FrontR, BackL, BackR;
    private DcMotorEx arm, pivot;
    private DcMotorEx leftLift, rightLift;
    private CRServo intake;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private DriveCommand driveCommand;

    private ArmSubsystem armSubsystem;
    private ArmExtendCommand armExtendCommand;
    private ArmRetractCommand armRetractCommand;

    private PivotSubsystem pivotSubsystem;
    private PivotRaiseCommand pivotRaiseCommand;
    private PivotLowerCommand pivotLowerCommand;

    private IntakeSubsystem intakeSubsystem;
    private IntakeForwardCommand intakeForwardCommand;
    private IntakeBackwardCommand intakeBackwardCommand;

    private LiftSubsystem liftSubsystem;
    private LiftExtendCommand liftExtendCommand;
    private LiftRetractCommand liftRetractCommand;

    private TelemetryScheduler telemetryScheduler;
    private TelemetryCommand telemetryCommand;

    @Override
    public void initialize(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // initialize gamepads and hardware components
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        FrontL = hardwareMap.get(DcMotorEx.class, "fl(eet footwork)");
        FrontR = hardwareMap.get(DcMotorEx.class, "fr(ank)");
        BackL = hardwareMap.get(DcMotorEx.class, "bl(itzcrank)");
        BackR = hardwareMap.get(DcMotorEx.class, "br(iar)");

        FrontL.setDirection(DcMotor.Direction.REVERSE);
        FrontR.setDirection(DcMotor.Direction.FORWARD);
        BackL.setDirection(DcMotor.Direction.REVERSE);
        BackR.setDirection(DcMotor.Direction.FORWARD);

        FrontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        BHI260IMU.Parameters parameters = new IMU.Parameters( new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initialize(parameters);
        imu.resetYaw();

        arm = hardwareMap.get(DcMotorEx.class, "Arm");
        pivot = hardwareMap.get(DcMotorEx.class, "Pivot");
        intake = hardwareMap.get(CRServo.class, "spinnything");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift.setDirection(DcMotorEx.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.FORWARD);

        // initialize subsystems and commands
        armSubsystem = new ArmSubsystem(arm);
        armExtendCommand = new ArmExtendCommand(armSubsystem, () -> operator.isDown(GamepadKeys.Button.LEFT_BUMPER), () -> operator.isDown(GamepadKeys.Button.RIGHT_BUMPER));
        armRetractCommand = new ArmRetractCommand(armSubsystem, () -> operator.isDown(GamepadKeys.Button.LEFT_BUMPER), () -> operator.isDown(GamepadKeys.Button.RIGHT_BUMPER));

        pivotSubsystem = new PivotSubsystem(pivot);
        pivotRaiseCommand = new PivotRaiseCommand(pivotSubsystem, () -> operator.isDown(GamepadKeys.Button.RIGHT_BUMPER));
        pivotLowerCommand = new PivotLowerCommand(pivotSubsystem, () -> operator.isDown(GamepadKeys.Button.RIGHT_BUMPER));

        intakeSubsystem = new IntakeSubsystem(intake);
        intakeForwardCommand = new IntakeForwardCommand(intakeSubsystem);
        intakeBackwardCommand = new IntakeBackwardCommand(intakeSubsystem);

        liftSubsystem = new LiftSubsystem(leftLift, rightLift);
        liftExtendCommand = new LiftExtendCommand(liftSubsystem, () -> driver.isDown(GamepadKeys.Button.LEFT_BUMPER));
        liftRetractCommand = new LiftRetractCommand(liftSubsystem, () -> driver.isDown(GamepadKeys.Button.LEFT_BUMPER));

        // link commands to buttons

        //arm
        (new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT)).whileHeld(armExtendCommand);
        (new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT)).whileHeld(armRetractCommand);

        // pivot
        (new GamepadButton(operator, GamepadKeys.Button.DPAD_UP)).whileHeld(pivotRaiseCommand);
        (new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN)).whileHeld(pivotLowerCommand);

        // intake
        (new GamepadButton(operator, GamepadKeys.Button.B)).whileHeld(intakeForwardCommand);
        (new GamepadButton(operator, GamepadKeys.Button.X)).whileHeld(intakeBackwardCommand);

        // lifts
        (new GamepadButton(driver, GamepadKeys.Button.Y)).whileHeld(liftExtendCommand);
        (new GamepadButton(driver, GamepadKeys.Button.A)).whileHeld(liftRetractCommand);

        //driving
        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);

        // running individual motors

        // mecanum driving
        driveCommand = new DriveCommand(driveSubsystem, () -> driver.getLeftX(), () -> driver.getLeftY(),
                () -> driver.getRightX(), () -> driver.isDown(GamepadKeys.Button.RIGHT_BUMPER));

        register(driveSubsystem);
        driveSubsystem.setDefaultCommand(driveCommand);

        // telemetry
        telemetryScheduler = new TelemetryScheduler(telemetry);
        telemetryCommand = new TelemetryCommand(telemetryScheduler,
                new Pair[]{
                        new Pair<String, DoubleSupplier>("Arm position", () -> arm.getCurrentPosition()),
                        new Pair<String, DoubleSupplier>("Arm target position", () -> arm.getTargetPosition()),
                        new Pair<String, DoubleSupplier>("Pivot position", () -> pivot.getCurrentPosition()),
                        new Pair<String, DoubleSupplier>("Left Lift position", () -> leftLift.getCurrentPosition()),
                        new Pair<String, DoubleSupplier>("Right lift position", () -> rightLift.getCurrentPosition()),
                        new Pair<String, DoubleSupplier>("Average encoder distance", () -> driveSubsystem.getAverageEncoderDistance()),
                        new Pair<String, DoubleSupplier>("Front Left power", () -> FrontL.getPower()),
                        new Pair<String, DoubleSupplier>("Front Right power", () -> FrontR.getPower()),
                        new Pair<String, DoubleSupplier>("Back Left power", () -> BackL.getPower()),
                        new Pair<String, DoubleSupplier>("Back Right power", () -> BackR.getPower())
                },

                new Pair[]{
                        new Pair<String, BooleanSupplier>("test", () -> FrontL.isMotorEnabled())
                },

                new Pair[]{
                        new Pair<String, String>("test", "test")
                }
        );
        register(telemetryScheduler);
        telemetryScheduler.setDefaultCommand(telemetryCommand);

    }

}
