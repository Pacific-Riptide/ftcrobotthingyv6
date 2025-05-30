package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp
public final class HookTest extends Robot {

    //Start
    @Override
    public void runOpMode() throws InterruptedException {
        super.initializeRobot();
        super.setRobotBehavior();
        waitForStart();

        while (opModeIsActive()) {
                double leftHookPosition = leftHook.getPosition();
                if (gamepad1.dpad_up||gamepad2.dpad_up) {
                    leftHookPosition += 0.1;
                } else if (gamepad1.dpad_down||gamepad2.dpad_down) {
                    leftHookPosition -= 0.1;
                }
                leftHook.setPosition(leftHookPosition);

                double rightHookPosition = rightHook.getPosition();
                if (gamepad1.dpad_up||gamepad2.dpad_up) {
                    rightHookPosition += 0.1;
                } else if (gamepad1.dpad_down||gamepad2.dpad_down) {
                    rightHookPosition -= 0.1;
                }
                rightHook.setPosition(rightHookPosition);

            telemetry.addData("Left / Right Hook Position", leftHook.getPosition() + "&" + rightHook.getPosition());
            telemetry.update();

            }

        }
    }





