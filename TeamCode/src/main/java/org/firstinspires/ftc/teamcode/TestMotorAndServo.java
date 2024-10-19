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
    private DcMotorEx motor1, motor2;
    private double totalCurrent1 = 0, maxCurrent1 = 0, minCurrent1 = 0;
    private double totalCurrent2 = 0, maxCurrent2 = 0, minCurrent2 = 0;
    private int count1 = 0, count2 = 0;

    @Override
    public void runOpMode() {

        waitForStart();
        servo = hardwareMap.get(CRServo.class, "servo");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        while(opModeIsActive()){
            if(gamepad1.x){
                motor1.setPower(1);
                telemetry.addLine("Motor1 power: 1");
            }
            else if(gamepad1.b){
                motor1.setPower(-1);
                telemetry.addLine("Motor1 power: -1");
            }
            else{
                motor1.setPower(0);
                telemetry.addLine("Motor1 power: 0");
            }
            if(gamepad1.dpad_up){
                motor2.setPower(1);
                telemetry.addLine("Motor2 power: 1");
            }
            else if(gamepad1.dpad_down){
                motor2.setPower(-1);
                telemetry.addLine("Motor2 power: -1");
            }
            else{
                motor2.setPower(0);
                telemetry.addLine("Motor2 power: 0");
            }
            if(gamepad1.y){
                servo.setPower(1);
                telemetry.addLine("Servo power: 1");
            }
            else if(gamepad1.a){
                servo.setPower(-1);
                telemetry.addLine("Servo power: -1");
            }
            else{
                servo.setPower(0);
                telemetry.addLine("Servo power: 0");
            }
            double current1 = motor1.getCurrent(CurrentUnit.AMPS);
            double current2 = motor2.getCurrent(CurrentUnit.AMPS);
            totalCurrent1 += current1;
            totalCurrent2 += current2;
            if(current1 > 0.01) {
                count1++;
                if(count1 == 1 || current1 > maxCurrent1){
                    maxCurrent1 = current1;
                }
                if(count1 == 1 || current1 < minCurrent1){
                    minCurrent1 = current1;
                }
            }
            if(current2 > 0.01) {
                count2++;
                if(count2 == 1 || current2 > maxCurrent2){
                    maxCurrent2 = current2;
                }
                if(count2 == 1 || current2 < minCurrent2){
                    minCurrent2 = current2;
                }
            }


            telemetry.addData("Motor1 current", current1);
            telemetry.addData("Min Motor1 current", minCurrent1);
            telemetry.addData("Max Motor1 current", maxCurrent1);
            telemetry.addData("Avg Motor1 current", (totalCurrent1) / count1);

            telemetry.addData("Motor2 current", current1);
            telemetry.addData("Min Motor2 current", minCurrent1);
            telemetry.addData("Max Motor2 current", maxCurrent1);
            telemetry.addData("Avg Motor2 current", (totalCurrent1) / count1);
            telemetry.update();
        }
    }
}
