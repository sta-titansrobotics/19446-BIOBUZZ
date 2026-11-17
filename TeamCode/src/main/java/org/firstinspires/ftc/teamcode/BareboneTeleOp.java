package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class BareboneTeleOp extends LinearOpMode {

    private static final double DRIVE_SCALE = 1.0;
    private static final double STRAFE_SCALE = 1.0;
    private static final double TURN_SCALE = 0.75;
    private static final double FRONT_LEFT_SCALE = 0.95;
    private static final double MAX_VELOCITY_SCALE = 1.0;

    // Replace these placeholders with measured/tuned values before enabling velocity PIDF.
    private static final boolean ENABLE_VELOCITY_PIDF = false;
    private static final double VELOCITY_P = 0.0;
    private static final double VELOCITY_I = 0.0;
    private static final double VELOCITY_D = 0.0;
    private static final double VELOCITY_F = 0.0;

    // Motors
    private DcMotorEx frontLeft;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private DcMotorEx frontRight;
    private DcMotor intake;
    public NormalizedColorSensor colorSensor;

    // Slew Rate Limiter Settings
    // 4.0 means 0 to 100% power takes 0.25 seconds (1.0 / 4.0).
    // Increase for snappier response; decrease for smoother acceleration.
    private static final double SLEW_RATE = 4.0;
    private double prevForward = 0;
    private double prevStrafe = 0;
    private double prevTurn = 0;

    static final double TICKS_PER_REV = 537.7;
    private int lastFLPos = 0, lastFRPos = 0, lastBLPos = 0, lastBRPos = 0;
    private double flRPM = 0, frRPM = 0, blRPM = 0, brRPM = 0;

    @Override
    public void runOpMode() {
        // Initialize
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        intake = hardwareMap.get(DcMotor.class, "intake");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");

        // Set Motor Direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // When there is no power, motors will start resisting
        for (DcMotorEx motor : getDriveMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Reset the encoder tick counters back to zero
        for (DcMotorEx motor : getDriveMotors()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        // Start in manual power mode, or encoder velocity mode when tuning is enabled.
        for (DcMotorEx motor : getDriveMotors()) {
            motor.setMode(ENABLE_VELOCITY_PIDF
                    ? DcMotor.RunMode.RUN_USING_ENCODER
                    : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (ENABLE_VELOCITY_PIDF) {
                motor.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
            }
        }

        // Intake
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setPower(0);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        ElapsedTime rpmTimer = new ElapsedTime();
        ElapsedTime slewTimer = new ElapsedTime();

        colorSensor.setGain(20.0f);

        waitForStart();
        rpmTimer.reset();
        slewTimer.reset();

        float[] hsvValues = new float[3];


        // 3. Run continuously while the match is active
        while (opModeIsActive()) {

            // Color Sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];

            String detectedColor = "";

            if (hue >= 40 && hue < 85) {
                detectedColor = "Yellow";
            } else if (hue > 330 || hue < 30) {
                detectedColor = "Red";
            } else if (hue >= 180 && hue <= 225){
                detectedColor = "Blue";
            } else {
                detectedColor = "Cannot Detect";
            }

            // Calculate time elapsed since last loop for slew rate
            double dt = slewTimer.seconds();
            slewTimer.reset();

            // Read joystick values (Gamepad 1)
            // Note: Gamepad Y-sticks are naturally negative when pushed forward, so we negate (-).
            double forwardBack;
            double strafe;
            double turn;
            double leftFrontPower;
            double rightFrontPower;
            double leftBackPower;
            double rightBackPower;

            if (gamepad2.a) {
                intake.setPower(-1.0);
            } else if (gamepad2.y) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }

            double rawForward = -gamepad1.left_stick_y * DRIVE_SCALE;
            double rawStrafe = gamepad1.left_stick_x * STRAFE_SCALE;
            double rawTurn = gamepad1.right_stick_x * TURN_SCALE;

            // Apply slew rate limiter to smooth out sudden stick movements
            forwardBack = slewLimit(rawForward, prevForward, SLEW_RATE, dt);
            strafe = slewLimit(rawStrafe, prevStrafe, SLEW_RATE, dt);
            turn = slewLimit(rawTurn, prevTurn, SLEW_RATE, dt);

            // Save current values for the next loop
            prevForward = forwardBack;
            prevStrafe = strafe;
            prevTurn = turn;

            // Apply power to the motors
            leftFrontPower = (forwardBack + strafe + turn) * FRONT_LEFT_SCALE;
            rightFrontPower = (forwardBack - strafe) - turn;
            leftBackPower = (forwardBack - strafe) + turn;
            rightBackPower = (forwardBack + strafe) - turn;

            double maxPower = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower)),
                            Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))
                    )
            );

            setWheelOutput(frontLeft, leftFrontPower / maxPower);
            setWheelOutput(frontRight, rightFrontPower / maxPower);
            setWheelOutput(backLeft, leftBackPower / maxPower);
            setWheelOutput(backRight, rightBackPower / maxPower);

            double deltaTime = rpmTimer.seconds();
            if (deltaTime >= 0.05) {
                int curFLPos = frontLeft.getCurrentPosition();
                int curFRPos = frontRight.getCurrentPosition();
                int curBLPos = backLeft.getCurrentPosition();
                int curBRPos = backRight.getCurrentPosition();

                flRPM = calculateRPM(curFLPos - lastFLPos, deltaTime);
                frRPM = calculateRPM(curFRPos - lastFRPos, deltaTime);
                blRPM = calculateRPM(curBLPos - lastBLPos, deltaTime);
                brRPM = calculateRPM(curBRPos - lastBRPos, deltaTime);

                lastFLPos = curFLPos;
                lastFRPos = curFRPos;
                lastBLPos = curBLPos;
                lastBRPos = curBRPos;

                rpmTimer.reset();

            }

            telemetry.addData("Front Left RPM", "%.1f", flRPM);
            telemetry.addData("Front Right RPM", "%.1f", frRPM);
            telemetry.addData("Back Left RPM", "%.1f", blRPM);
            telemetry.addData("Back Right RPM", "%.1f", brRPM);
            telemetry.addData("Detected Object Color", detectedColor);
            telemetry.addData("Drive control", ENABLE_VELOCITY_PIDF
                    ? "Encoder velocity PIDF"
                    : "Open-loop power");
            if (ENABLE_VELOCITY_PIDF) {
                telemetry.addData("Wheel velocity (ticks/s)", "FL %.0f, FR %.0f, BL %.0f, BR %.0f",
                        frontLeft.getVelocity(), frontRight.getVelocity(),
                        backLeft.getVelocity(), backRight.getVelocity());
                telemetry.addData("PIDF", "P %.3f I %.3f D %.3f F %.3f",
                        VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
            }

            telemetry.update();
        }
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

    private double calculateRPM(int deltaTicks, double deltaTimeSeconds){
        double ticksPerSecond = (double) deltaTicks / deltaTimeSeconds;
        return (ticksPerSecond / TICKS_PER_REV) * 60;

    }

    private double slewLimit(double target, double current, double rateLimit, double dt) {
        double maxChange = rateLimit * dt;
        double delta = target - current;

        if (Math.abs(delta) > maxChange) {
            return current + Math.signum(delta) * maxChange;
        }
        return target;
    }

}