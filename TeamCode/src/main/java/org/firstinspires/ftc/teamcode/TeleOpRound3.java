package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public final class TeleOpRound3 extends LinearOpMode {
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor armMotor;
    private DcMotor rotaMotor;
    private Servo clawServo;
    private Servo clawRota;

    private static final double HOLD_POWER = 0.0;   // Power to hold position
    private static final double UP_POWER = -1;     // Full power for upward movement
    private static final double DOWN_POWER = 1;  // Limited power for controlled descent

    private static final double DEROTA_POWER = 0.0;   // Power to hold position
    private static final double ROTA_POWER = -1;// Full power for ascension
    private static final double OROTA_POWER = 1;// Full power for ascension

    private static final double clawMinPosition = 0.0;
    private static final double clawMaxPosition = 1.0;

    private static final double clawRotatorMinPosition = 0.0;
    private static final double clawRotatorMaxPosition = 1.0;

    @Override
    public void runOpMode() {
        initializeRobot();
        setRobotBehavior();
        waitForStart();

        while (opModeIsActive()) {
            // Operate drive train
            operateDriveTrain();

            //Operate Viper Slide Up and Down
            operateViperSlide();

            operateClaw(clawMinPosition, clawMaxPosition);

            operateClawRotator(clawRotatorMinPosition, clawRotatorMaxPosition);

            // Telemetry for debugging
            telemetry.addData("Front Left Power", frontLeftMotor.getPower());
            telemetry.addData("Back Left Power", backLeftMotor.getPower());
            telemetry.addData("Front Right Power", frontRightMotor.getPower());
            telemetry.addData("Back Right Power", backRightMotor.getPower());
            telemetry.addData("Arm Power", armMotor.getPower());
            telemetry.addData("Rotation Position", rotaMotor.getCurrentPosition());
            telemetry.addData("Rotation Power", rotaMotor.getPower());

            telemetry.addData("Claw Position", clawServo.getPosition());
            telemetry.addData("Claw Rotator Position", clawRota.getPosition());
            telemetry.update();
        }
    }

    private void operateClawRotator(Double minPosition, Double maxPosition) {
        double clawRotaPosition = clawRota.getPosition();
        if(gamepad1.dpad_left){
            clawRotaPosition += 0.003;
        } else if (gamepad1.dpad_right) {
            clawRotaPosition -= 0.003;
        }

        clawRotaPosition = Math.max(minPosition, Math.min(clawRotaPosition, maxPosition));
        clawRota.setPosition(clawRotaPosition);
    }

    private void operateClaw(Double minPosition, Double maxPosition) {
        double clawPosition = clawServo.getPosition();

        if(gamepad1.a){
            clawPosition += 0.003;
        } else if (gamepad1.b) {
            clawPosition -= 0.003;
        }
        clawPosition = Math.max(minPosition, Math.min(clawPosition, maxPosition));
        clawServo.setPosition(clawPosition);
    }

    private void operateViperSlide() {
        // Check for button inputs
        boolean downPressed = gamepad1.left_bumper;  // Use left bumper for down
        boolean upPressed = gamepad1.right_bumper; // Use right bumper for up

        if (upPressed) {
            // Move arm up
            armMotor.setPower(UP_POWER);
        } else if (downPressed) {
            // Move arm down
            armMotor.setPower(DOWN_POWER);
        } else {
            // Hold position
            armMotor.setPower(HOLD_POWER);
        }

        // Check for button inputs
        boolean derotaPressed = gamepad1.dpad_down;  // Use left bumper for down
        boolean arotaPressed = gamepad1.dpad_up;// Use right bumper for up



        if (arotaPressed) {
            // Rotate up
            rotaMotor.setPower(ROTA_POWER);
            //rotaMotor.setTargetPosition(1000);
            //rotaMotor.setPower(-1);
            //rotaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if (derotaPressed) {
            // Rotate down
            rotaMotor.setPower(OROTA_POWER);
            //rotaMotor.setTargetPosition(100);
            //rotaMotor.setPower(-1);
            //rotaMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else {
            // Hang position
            rotaMotor.setPower(DEROTA_POWER);
        }
    }

    private void operateDriveTrain() {
        double drive = -gamepad1.left_stick_y; // Forward/Backward
        double strafe = gamepad1.left_stick_x; // Left/Right
        double turn = gamepad1.right_stick_x;  // Turning

        // Calculate power for each motor
        double frontLeftPower = drive + strafe + turn;
        double backLeftPower = drive - strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backRightPower = drive + strafe - turn;

        // Normalize motor powers if any power is above 1.0
        double maxPower = Math.max(1.0, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));

        frontLeftPower /= maxPower;
        backLeftPower /= maxPower;
        frontRightPower /= maxPower;
        backRightPower /= maxPower;

        // Set power to motors
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    private void setRobotBehavior() {
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotaMotor.setDirection(DcMotor.Direction.REVERSE);
        rotaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawRota.setDirection(Servo.Direction.REVERSE);
    }

    private void initializeRobot() {
        // Declare motors and servos
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
