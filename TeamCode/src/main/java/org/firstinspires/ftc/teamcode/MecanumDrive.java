package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive", group = "Drive")
public class MecanumDrive extends OpMode {
    private static final double DRIVE_SCALE = 1.0;
    private static final double STRAFE_SCALE = 1.0;
    private static final double TURN_SCALE = 1.0;
    private static final double MAX_VELOCITY_SCALE = 1.0;

    // Replace these placeholders with measured/tuned values before enabling velocity PIDF.
    private static final boolean ENABLE_VELOCITY_PIDF = false;
    private static final double VELOCITY_P = 0.0;
    private static final double VELOCITY_I = 0.0;
    private static final double VELOCITY_D = 0.0;
    private static final double VELOCITY_F = 0.0;

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotorEx.class, "front_left");
        frontRight = hardwareMap.get(DcMotorEx.class, "front_right");
        backLeft = hardwareMap.get(DcMotorEx.class, "back_left");
        backRight = hardwareMap.get(DcMotorEx.class, "back_right");

        // Common FTC drivetrain setup; reverse individual directions if the
        // robot's motor mounting differs.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : getDriveMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(ENABLE_VELOCITY_PIDF
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (ENABLE_VELOCITY_PIDF) {
                motor.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
            }
        }
    }

    @Override
    public void loop() {
        double drive = -gamepad1.left_stick_y * DRIVE_SCALE;
        double strafe = gamepad1.left_stick_x * STRAFE_SCALE;
        double turn = gamepad1.right_stick_x * TURN_SCALE;

        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                )
        );

        setWheelOutput(frontLeft, frontLeftPower / maxPower);
        setWheelOutput(frontRight, frontRightPower / maxPower);
        setWheelOutput(backLeft, backLeftPower / maxPower);
        setWheelOutput(backRight, backRightPower / maxPower);

        telemetry.addData("Control mode", ENABLE_VELOCITY_PIDF ? "Encoder velocity PIDF" : "Open-loop power");
        telemetry.addData("Drive / Strafe / Turn", "%.2f / %.2f / %.2f", drive, strafe, turn);
        telemetry.addData("Wheel commands", "FL %.2f, FR %.2f, BL %.2f, BR %.2f",
                frontLeftPower / maxPower, frontRightPower / maxPower,
                backLeftPower / maxPower, backRightPower / maxPower);
        if (ENABLE_VELOCITY_PIDF) {
            telemetry.addData("Wheel velocity (ticks/s)", "FL %.0f, FR %.0f, BL %.0f, BR %.0f",
                    frontLeft.getVelocity(), frontRight.getVelocity(),
                    backLeft.getVelocity(), backRight.getVelocity());
            telemetry.addData("PIDF", "P %.3f I %.3f D %.3f F %.3f",
                    VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
        }
        telemetry.update();
    }

    private void setWheelOutput(DcMotorEx motor, double normalizedCommand) {
        if (ENABLE_VELOCITY_PIDF) {
            double maxTicksPerSecond = motor.getMotorType().getAchieveableMaxTicksPerSecond();
            motor.setVelocity(normalizedCommand * maxTicksPerSecond * MAX_VELOCITY_SCALE);
        } else {
            motor.setPower(normalizedCommand);
        }
    }

    private DcMotorEx[] getDriveMotors() {
        return new DcMotorEx[] { frontLeft, frontRight, backLeft, backRight };
    }
}
