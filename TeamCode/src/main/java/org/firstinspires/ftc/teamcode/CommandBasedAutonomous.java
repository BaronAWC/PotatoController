package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@Autonomous(name="CommandBase Autonomous")
public class CommandBasedAutonomous extends CommandOpMode {

    private DcMotorEx FrontL, FrontR, BackL, BackR;
    private DcMotorEx arm, pivot;
    private DcMotorEx leftLift, rightLift;
    private CRServo intake;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;

    private PivotSubsystem pivotSubsystem;

    private IntakeSubsystem intakeSubsystem;

    private LiftSubsystem liftSubsystem;

    @Override
    public void initialize(){

        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

        leftLift = hardwareMap.get(DcMotorEx.class, "Left Lift");
        rightLift = hardwareMap.get(DcMotorEx.class, "Right Lift");

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        armSubsystem = new ArmSubsystem(arm);

        pivotSubsystem = new PivotSubsystem(pivot);

        intakeSubsystem = new IntakeSubsystem(intake);

        liftSubsystem = new LiftSubsystem(leftLift, rightLift);

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);

        // schedule all commands in this method
        waitForStart();
        new SequentialCommandGroup(
                //new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(2000),
                //new DriveDistanceCommand(driveSubsystem, 50, 30, -0.1, telemetry)
                //new DriveRotateCommand(driveSubsystem, -90, 0.1, telemetry)
                new DriveDistanceCommand(driveSubsystem, 20, 0, 0.1, telemetry),
                new DriveDistanceCommand(driveSubsystem, 20, 90, 0.1, telemetry),
                new DriveDistanceCommand(driveSubsystem, 20, -90, 0.1, telemetry)
                //new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 5, 30, 0.3, telemetry)
                //        new DriveRotateCommand(driveSubsystem, 60, 0.3, telemetry))
                //new ArmRunToPositionCommand(armSubsystem, telemetry,-3000, 1),
                //new PivotRunToPositionCommand(pivotSubsystem, -3600, 0.5)
        ).schedule();

//        new SequentialCommandGroup(
//                new ArmRunToPositionCommand(armSubsystem, -3000)
//        ).schedule();
    }
}
