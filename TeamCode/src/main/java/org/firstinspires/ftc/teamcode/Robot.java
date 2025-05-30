package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class Robot extends LinearOpMode {
    protected DcMotor frontLeftMotor;
    protected DcMotor backLeftMotor;
    protected DcMotor frontRightMotor;
    protected DcMotor backRightMotor;
    protected static DcMotor armMotor;
    protected static DcMotor rotaMotor;
    protected static Servo clawServo;
    protected static Servo clawRota;
    protected static Servo claw360;
    protected static Servo leftHook;
    protected static Servo rightHook;
  //  protected static DistanceSensor distanceSensor;
    //protected static Servo inputSlide;
    //protected static Servo inputRota;
    //protected static Servo input360;
    //protected static Servo inputRoller;
    protected TouchSensor touchSensor;
    protected IMU imu;

    public void initializeRobot() {
        // Declare motors and servos
        frontLeftMotor = hardwareMap.dcMotor.get("TopLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("BottomLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("TopRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("BottomRightMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        rotaMotor = hardwareMap.dcMotor.get("rotaMotor");
        clawServo = hardwareMap.servo.get("ClawServo");
        clawRota = hardwareMap.servo.get("ClawRota");
        claw360 = hardwareMap.servo.get("Claw360");
        leftHook = hardwareMap.servo.get("leftHook");
        rightHook = hardwareMap.servo.get("rightHook");
 //       distanceSensor = hardwareMap.get(DistanceSensor.class, "magneticSensor");
       // inputSlide = hardwareMap.servo.get("Inputslide");
       // inputRota = hardwareMap.servo.get("InputRota");
      //  input360 = hardwareMap.servo.get("Input360");
       // inputRoller = hardwareMap.servo.get("Inputroller");
        touchSensor = hardwareMap.touchSensor.get("touchSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);
    }

    public void setRobotBehavior() {
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotaMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotaMotor.setDirection(DcMotor.Direction.REVERSE);
        rotaMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        clawRota.setDirection(Servo.Direction.REVERSE);
        leftHook.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        throw new InterruptedException("Operation not supported at this level");
    }

    public void displayTelemetry(){
        // Telemetry for debugging
        telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        telemetry.addData("Rotation Position", rotaMotor.getCurrentPosition());
        telemetry.addData("Left / Right Hook Position", leftHook.getPosition() + "&" + rightHook.getPosition());

        telemetry.addData("Claw Position", clawServo.getPosition());
        telemetry.addData("Claw Rotator Position", clawRota.getPosition());
        telemetry.addData("Claw Circumlocution Position", claw360.getPosition());
      //  telemetry.addData("Input Slide Position", inputSlide.getPosition());
       // telemetry.addData("Input Rotator Position", inputRota.getPosition());
      //  telemetry.addData("Input Circumlocution Position", input360.getPosition());
      //  telemetry.addData("Input Roller Position", inputRoller.getPosition());

        telemetry.addData("Front Left Motor Ticks", frontLeftMotor.getCurrentPosition());
        telemetry.addData("Front Right Motor Ticks", frontRightMotor.getCurrentPosition());
        telemetry.addData("Back Left Motor Ticks", backLeftMotor.getCurrentPosition());
        telemetry.addData("Back Right Motor Ticks", backRightMotor.getCurrentPosition());

        telemetry.addData("Robot Yaw", imu.getRobotYawPitchRollAngles().getYaw()) ;
        telemetry.addData("Robot Pitch", imu.getRobotYawPitchRollAngles().getPitch()) ;
        telemetry.addData("Robot X", imu.getRobotOrientationAsQuaternion().x) ;
        telemetry.addData("Robot Y", imu.getRobotOrientationAsQuaternion().y);
        telemetry.addData("Robot Z", imu.getRobotOrientationAsQuaternion().z);

        telemetry.update();
    }
}
