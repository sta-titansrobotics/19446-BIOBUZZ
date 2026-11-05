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

        telemetry.addData("Status", "Initialized and Ready");
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

            telemetry.addData("Drive Motors", "FL: %.2f | FR: %.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Drive Motors", "BL: %.2f | BR: %.2f", leftBackPower, rightBackPower);

            telemetry.update();
        }
    }
}