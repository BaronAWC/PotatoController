package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
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
                //drop off piece
//                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 55, -45, 0.4, telemetry),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, -4000, 0.8),
//                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.5)),
//                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 45, 0.25, telemetry),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND, 0.8)),
//                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 27, 0, 0.3, telemetry),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND, 0.8)),
//                new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 800, 0.5),
//                        new DriveDistanceCommand(driveSubsystem, 3, -90, 0.4, telemetry)),
//                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(2000),
//
//                // park by submersible
//                new ParallelCommandGroup(new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 200, 0.5),
//                        new DriveDistanceCommand(driveSubsystem, 14, 90, 0.3, telemetry)),
//                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 54, 0, -0.4, telemetry),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND, 1)),
//                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 90, 0.25, telemetry),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND / 2, 0.75)),
//                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 100, 0, -0.6, telemetry), // driving down
//                    new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 0.75)),
//                new ParallelCommandGroup(new DriveRotateCommand(driveSubsystem, 0, 0.25, telemetry),
//                    new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS / 2, 0.5)),
//                new ParallelCommandGroup(new DriveDistanceCommand(driveSubsystem, 40, 0, -0.6, telemetry),
//                        new PivotRunToPositionCommand(pivotSubsystem, 0, 0.5)),
//                        new DriveStopCommand(driveSubsystem, telemetry)

                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 33.75, -30, 45, 0.65, 0.65, telemetry)
                        //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.9),
                        //new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.LIMITED_EXTEND, 0.9)
                ),
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 3, 0, 0, 0.5, 0, telemetry)
                        //new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND - 200, 0.9)
                ),
                //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS - 1000, 0.5),
                new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.Out).withTimeout(1500),
                //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS- 300, 0.9),
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 20, 0, 0, -0.65, -0.65, telemetry)
                        //new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.PICKUP, 0.9),
                        //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 500, 0.9)
                ),
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 8, -45, 0, 0.5, 0, telemetry)
                        //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 200, 0.9)
                ),
                new ParallelCommandGroup(
                        //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.LOWEST_POS + 50, 0.9)
                        //new IntakeRunCommand(intakeSubsystem, IntakeRunCommand.Direction.In).withTimeout(1500)
                ),
                new ParallelCommandGroup(
                        new AutoDriveCommand(driveSubsystem, 40, 60, 45, 0.5, 0.3, telemetry)
                        //new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.9),
                        //new ArmRunToPositionCommand(armSubsystem, telemetry, ArmSubsystem.FULL_EXTEND - 200, 0.9)
                )
//                new ParallelCommandGroup(
//                        new PivotRunToPositionCommand(pivotSubsystem, 0, 0.75),
//                        new ArmRunToPositionCommand(armSubsystem, telemetry, 0, 0.75)
//                )
        ).schedule();
    }
}
