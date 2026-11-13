package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

        ElapsedTime rpmTimer = new ElapsedTime();
        ElapsedTime slewTimer = new ElapsedTime();

        waitForStart();
        rpmTimer.reset();
        slewTimer.reset();

        // 3. Run continuously while the match is active
        while (opModeIsActive()) {

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

                    // Reset slew history while locked so it ramps up smoothly on release
                    prevForward = 0;
                    prevStrafe = 0;
                    prevTurn = 0;

                    isBraking = true;

                }
            } else {

                isBraking = false;

                // Restore manual joystick mode to all 4 motors
                frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double rawForward = -gamepad1.left_stick_y;
                double rawStrafe = gamepad1.left_stick_x;
                double rawTurn = gamepad1.right_stick_x * 0.75f;

                // Apply slew rate limiter to smooth out sudden stick movements
                forwardBack = slewLimit(rawForward, prevForward, SLEW_RATE, dt);
                strafe = slewLimit(rawStrafe, prevStrafe, SLEW_RATE, dt);
                turn = slewLimit(rawTurn, prevTurn, SLEW_RATE, dt);

                // Save current values for the next loop
                prevForward = forwardBack;
                prevStrafe = strafe;
                prevTurn = turn;

                // Apply power to the motors
                leftFrontPower = forwardBack + strafe + turn;
                rightFrontPower = (forwardBack - strafe) - turn;
                leftBackPower = (forwardBack - strafe) + turn;
                rightBackPower = (forwardBack + strafe) - turn;
                frontLeft.setPower(leftFrontPower * 0.95);
                frontRight.setPower(rightFrontPower);
                backLeft.setPower(leftBackPower);
                backRight.setPower(rightBackPower);
            }

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

            telemetry.addData("Lock Enabled?", isBraking);
            telemetry.addData("Front Left RPM", "%.1f", flRPM);
            telemetry.addData("Front Right RPM", "%.1f", frRPM);
            telemetry.addData("Back Left RPM", "%.1f", blRPM);
            telemetry.addData("Back Right RPM", "%.1f", brRPM);

            telemetry.update();
        }
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