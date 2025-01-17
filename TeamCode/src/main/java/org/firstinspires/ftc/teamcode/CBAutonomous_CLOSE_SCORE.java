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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="CB Autonomous CLOSE SCORE")
public class CBAutonomous_CLOSE_SCORE extends CommandOpMode {

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
                - second closest tile to bucket
                - facing bucket
                - left wheels against wall
                - back of the robot above inner groove of the tile
                 */

                // 1. drive up to the bucket
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AutoDriveCommand(driveSubsystem, 50, -30, 45, 0.5, 0.5, telemetry),
                                new AutoDriveCommand(driveSubsystem, 17, 0, 45, 0.25, 0.25, telemetry)
                        ),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 1),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND, 1)
                ),

                // 2. place first block
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 825, 0.65),
                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(1500),

                // 3. back away from the bucket and drive to second block
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 1),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AutoDriveCommand(driveSubsystem, 25, 0, 0, -0.5, 0.5, telemetry),
                                new AutoDriveCommand(driveSubsystem, 54, 73, 0, -0.5, 0.25, telemetry)
                        ),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.PICKUP, 1),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 1000, 1)
                ),

                // 4. pick up second block
                new ParallelCommandGroup(
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 75, 1),
                        new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(1250)
                ),

                // 5. drive up to bucket
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AutoDriveCommand(driveSubsystem, 40, 60, 45, 0.5, 0.5, telemetry),
                                new WaitCommand(1000),
                                new AutoDriveCommand(driveSubsystem, 33, 10, 45, 0.5, 0.5, telemetry)
                        ),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 1),
                        new SequentialCommandGroup(
                                new WaitCommand(2250),
                                new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND, 1)
                        )
                ),

                // 6. place second block -- copied from first
                // ****
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 825, 0.65),
                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(1500),

                // 7. back away from the bucket and drive in front of submersible
                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 1),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new AutoDriveCommand(driveSubsystem, 25, 0, 0, -0.5, 0.5, telemetry),
                                new AutoDriveCommand(driveSubsystem, 110, -90, 0, 1, 0.85, telemetry)
                        ),
                        new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 1)
                ),

                // 8. drive to submersible and touch
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 65, 0, 0, -0.5, 0, telemetry),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.AUTO_POS, 0.75)
                        )
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
