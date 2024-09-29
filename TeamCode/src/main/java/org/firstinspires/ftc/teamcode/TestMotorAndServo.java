package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name="Test Motor and Servo", group="Linear OpMode")
public class TestMotorAndServo extends LinearOpMode {

    private CRServo servo;
    private DcMotorEx motor;
    private double totalCurrent = 0, maxCurrent = 0, minCurrent = 0;
    private int count = 0;

    @Override
    public void runOpMode() {
        waitForStart();
        servo = hardwareMap.get(CRServo.class, "servo");
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        while(opModeIsActive()){
            if(gamepad1.x){
                motor.setPower(1);
            }
            else if(gamepad1.b){
                motor.setPower(-1);
            }
            else{
                motor.setPower(0);
            }
            if(gamepad1.y){
                servo.setPower(1);
            }
            else if(gamepad1.a){
                servo.setPower(-1);
            }
            else{
                servo.setPower(0);
            }
            double current = motor.getCurrent(CurrentUnit.AMPS);
            totalCurrent += current;
            if(current > 0.01) {
                count++;
                if(count == 1 || current > maxCurrent){
                    maxCurrent = current;
                }
                if(count == 1 || current < minCurrent){
                    minCurrent = current;
                }
            }


            telemetry.addData("Motor current", current);
            telemetry.addData("Min current", minCurrent);
            telemetry.addData("Max current", maxCurrent);
            telemetry.addData("Avg current", (totalCurrent) / count);
            telemetry.update();
        }
    }
}
