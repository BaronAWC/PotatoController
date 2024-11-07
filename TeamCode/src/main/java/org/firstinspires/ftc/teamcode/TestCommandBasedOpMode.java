package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.command.SubsystemBase;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="Test CommandBase OpMode")

public class TestOpMode extends CommandOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Chassis chassis = new Chassis(hardwareMap, telemetry);
        boolean isSlowMode = true;
        boolean prevXPressed = false;

        GamepadEx driver = new GamepadEx(gamepad1);
        GamepadEx operator = new GamepadEx(gamepad2);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while(opModeIsActive()) {
            // TODO: in aaron's version this determined if you were red or blue alliance
            int factor = 1;
            double y = -driver.getLeftY() * factor;
            double x = driver.getLeftX() * factor;
            double rx = driver.getRightX();

            // rising edge of x pressed
            isSlowMode = false;
            driver.getGamepadButton(GamepadKeys.Button.X)
                    .whenActive(new InstantCommand(() -> isSlowMode = true));
            //boolean isXPressed = gamepad1.x;
            // detecting the moment when button x is pressed
            // if (isXPressed && !prevXPressed) {
            //     isSlowMode = !isSlowMode; // toggle slow mode
            // }
            // prevXPressed = isXPressed; // update state

            chassis.gamepadDrive(y, x, rx, isSlowMode);

            operator.getGamepadButton(GamepadKeys.dpad_left)
                .and(operator.getGamepadButton(GamepadKeys.dpad_right).negate())
                .whenActive(new InstantCommand(() -> ));

        }

        /*while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }*/
    }
}
