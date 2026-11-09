package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class BareboneTeleOp extends LinearOpMode {

    // Motors
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    // Braking Mechanism
    private int lockFL = 0, lockFR = 0, lockBL = 0, lockBR = 0;
    private boolean isBraking = false;

    @Override
    public void runOpMode() {
        // Initialize
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Set Motor Direction
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);

        // When there is no power, motors will start resisting
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset the encoder tick counters back to zero
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Start out in normal manual driving mode
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        // 3. Run continuously while the match is active
        while (opModeIsActive()) {

            // Read joystick values (Gamepad 1)
            // Note: Gamepad Y-sticks are naturally negative when pushed forward, so we negate (-).
            float forwardBack;
            float strafe;
            float turn;
            float leftFrontPower;
            float rightFrontPower;
            float leftBackPower;
            float rightBackPower;

            if (gamepad1.right_bumper) {
                if(!isBraking) {
                    // Where robot is right now.
                    lockFL = frontLeft.getCurrentPosition();
                    lockFR = frontRight.getCurrentPosition();
                    lockBL = backLeft.getCurrentPosition();
                    lockBR = backRight.getCurrentPosition();

                    // Send those exact coordinates to each motor's controller
                    frontLeft.setTargetPosition(lockFL);
                    frontRight.setTargetPosition(lockFR);
                    backLeft.setTargetPosition(lockBL);
                    backRight.setTargetPosition(lockBR);

                    // Turn on internal positioning
                    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    // SAFE POWER LIMIT: Set to 0.35 so motors hold position
                    // without drawing too much battery current or stripping internal gears.
                    frontLeft.setPower(0.35);
                    frontRight.setPower(0.35);
                    backLeft.setPower(0.35);
                    backRight.setPower(0.35);

                    isBraking = true;
                }
            } else {

                isBraking = false;

                // Restore manual joystick mode to all 4 motors
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                forwardBack = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x;
                turn = gamepad1.right_stick_x * 0.75f;

                // Apply power to the motors
                leftFrontPower = forwardBack + strafe + turn;
                rightFrontPower = -((forwardBack - strafe) - turn);
                leftBackPower = (forwardBack - strafe) + turn;
                rightBackPower = (forwardBack + strafe) - turn;
                frontLeft.setPower(leftFrontPower);
                frontRight.setPower(-rightFrontPower);
                backLeft.setPower(leftBackPower);
                backRight.setPower(rightBackPower);
            }

            telemetry.addData("Lock Enabled?", isBraking);

            telemetry.update();
        }
    }
}