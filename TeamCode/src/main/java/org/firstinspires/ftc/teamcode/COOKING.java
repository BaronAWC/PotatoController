package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="CB Autonomous COOKING")
public class COOKING extends CommandOpMode {

    private DcMotorEx FrontL, FrontR, BackL, BackR;
    private DcMotorEx arm, pivot;
    private CRServo intakeFront, intakeBack;
    private BHI260IMU imu;
    private DriveSubsystem driveSubsystem;
    private ArmSubsystem armSubsystem;

    private PivotSubsystem pivotSubsystem;

    private IntakeSubsystem intakeSubsystem;

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
        intakeFront = hardwareMap.get(CRServo.class, "spinnything");
        intakeBack = hardwareMap.get(CRServo.class, "spinnything2");

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armSubsystem = new ArmSubsystem(arm);

        pivotSubsystem = new PivotSubsystem(pivot);

        intakeSubsystem = new IntakeSubsystem(intakeFront, intakeBack);

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);

        // schedule all commands in this method
        waitForStart();
        new SequentialCommandGroup(
                // starting on close side
                /*
                - third closest tile to observatory
                - facing submersible
                - back wheels against the wall
                - left of the robot above inner groove of the tile
                 */

                // 1. drive up and extend arm
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 80, 20, 0, 0.5, 0, telemetry),
                        new PivotRunToPositionCommand(pivotSubsystem, 2900, 0.3)
                ),
                new ArmRunToPositionCommand(armSubsystem, telemetry, -1400, 0.3),

                // 2. release specimen
                new ParallelCommandGroup(
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 1),
                        new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(1500)
                ),

                // 3. drive away and park
                new AutoDriveCommand(driveSubsystem, 20, 0, 0, -0.5, 0, telemetry),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AutoDriveCommand(driveSubsystem, 70, 70, 90, -0.5, 0.5, telemetry),
                                new AutoDriveCommand(driveSubsystem, 50, -45, 90, -0.5, 0.5, telemetry)
                        ),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 0.8),
                        new PivotRunToPositionCommand(pivotSubsystem, 0, 0.8)
                )
//
//
//                // reset the arm and pivot
//                new WaitCommand(5000),
//                new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 1),
//                new PivotRunToPositionCommand(pivotSubsystem, 0, 1)
        ).schedule();
    }
}
