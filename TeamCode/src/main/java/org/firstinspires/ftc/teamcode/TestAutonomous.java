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

@Autonomous(name="Test Auto")
public class TestAutonomous extends CommandOpMode {

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

                // Basic test:
//                new AutoDriveCommand(driveSubsystem, 30, 0, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -90, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 0, 0, -0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 90, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 0, 0, -90, 0, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 0, 0, 90, 0, 0.5, telemetry)

                // Diagonal test:
//                new AutoDriveCommand(driveSubsystem, 30, -45, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -45, 0, -0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -90, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 45, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 45, 0, -0.5, 0, telemetry)

                // Drive + Rotate test 1:
//                new AutoDriveCommand(driveSubsystem, 30, -45, 45,0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 90, 0, 0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -90, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 45, -45, 0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -90, 0, 0.5, 0.5, telemetry)

                // Drive + Rotate test 2:
//                new AutoDriveCommand(driveSubsystem, 30, 45, 45, 0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 0, 0, -0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 90, 0, 0.5, 0, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, -45, -45, 0.5, 0.5, telemetry),
//                new AutoDriveCommand(driveSubsystem, 30, 0, 0, -0.5, 0.5, telemetry)

                // Auto driving
                new AutoDriveCommand(driveSubsystem, 50, -30, 45, 0.5, 0.5, telemetry),
                new AutoDriveCommand(driveSubsystem, 14, 0, 45, 0.25, 0.25, telemetry),
                new AutoDriveCommand(driveSubsystem, 25, 0, 0, -0.5, 0.5, telemetry),
                new AutoDriveCommand(driveSubsystem, 50, 70, 0, -0.5, 0.25, telemetry),
                new AutoDriveCommand(driveSubsystem, 40, 60, 45, 0.5, 0.5, telemetry)

        ).schedule();
    }
}
