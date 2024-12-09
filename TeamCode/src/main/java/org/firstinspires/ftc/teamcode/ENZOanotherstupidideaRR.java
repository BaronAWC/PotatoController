org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
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





import org.firstinspires.ftc.teamcode.ArmRunToPositionCommand;
import org.firstinspires.ftc.teamcode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.DriveSubsystem;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.LiftSubsystem;
import org.firstinspires.ftc.teamcode.PivotRunToPositionCommand;
import org.firstinspires.ftc.teamcode.PivotSubsystem;


@Autonomous(name="idklmao")
public class EnzoanotherstupidideaRR extends CommandOpMode {


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

        armSubsystem = new ArmSubsystem(arm);

        pivotSubsystem = new PivotSubsystem(pivot);

        intakeSubsystem = new IntakeSubsystem(intake);

        liftSubsystem = new LiftSubsystem(leftLift, rightLift);

        driveSubsystem = new DriveSubsystem(FrontL, FrontR, BackL, BackR, imu);

        // schedule all commands in this method
        waitForStart();
        new SequentialCommandGroup(

        ).schedule();


        // Define starting pose
        Pose2d startPose = new Pose2d(-11, -63, Math.toRadians(180));




// Set the starting pose for the RoadRunner localizer
        drive.setPoseEstimate(startPose);

        Pose2d beginPose = new Pose2d(-11.00, -63, Math.toRadians(180.00));
        TrajectoryActionBuilder trajectory = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(-15, -59)) // Move left and up
                .splineToSplineHeading(new Pose2d(-57, -61, Math.toRadians(220)), Math.toRadians(220))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.45, -25.45, Math.toRadians(180.00)), Math.toRadians(90))
                .setReversed(false)
                .strafeToSplineHeading(new Pose2d(-62, -56, Math.toRadians(240)))
                .setReversed(true)
                .strafeToSplineHeading(new Pose2d(-36.45, -25.45, Math.toRadians(180.00)))
                .setReversed(false)
                .strafeToSplineHeading(new Pose2d(-62, -56, Math.toRadians(240)))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.45, -25.45, Math.toRadians(180.00)), Math.toRadians(90))
                .setReversed(false)
                .strafeToSplineHeading(new Pose2d(-62, -56, Math.toRadians(240)))
                .strafeToLinearHeading(new Pose2d(-22, 7, Math.toRadians(180.00)))
                .build();


        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                        new ArmRunToPositionCommand(armSubsystem, telemetry, -4000, 0.8),
                        new PivotRunToPositionCommand(pivotSubsystem, PivotSubsystem.HIGHEST_POS, 0.5)
                )
        );





    }
}





;
