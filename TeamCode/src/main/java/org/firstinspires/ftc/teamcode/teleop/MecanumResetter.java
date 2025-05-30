package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public final class MecanumResetter extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor rotaMotor;
    private Servo clawServo;
    private Servo clawRota;

    @Override
    public void runOpMode() {
        waitForStart();
        initializeRobot();
        setRobotBehavior();
        //Use this case if only robot needs hard reset
        //No using in competition
    }

    private void setRobotBehavior() {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotaMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void initializeRobot() {
        frontLeftMotor = hardwareMap.dcMotor.get("TopLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BottomLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("TopRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("BottomRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        rotaMotor = hardwareMap.dcMotor.get("rotaMotor");
        clawServo = hardwareMap.servo.get("ClawServo");
        clawRota = hardwareMap.servo.get("ClawRota");
    }
}
