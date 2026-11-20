package pedro_example;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.Set;

import biobuzz.subsystems.VelocityPID;
import biobuzz.tuning;

@Autonomous
public class OneTipParkRed extends LinearOpMode {
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();

    // Subsystems & Hardware
    private DcMotorEx intake;
    private DcMotorEx transfer;
    private VelocityPID leftshooterPID;
    private VelocityPID rightshooterPID;

    // Target velocity setting for flywheel
    private static final double TARGET_SHOOT_VELOCITY = 750.0;

    // Poses
    private final Pose startPose = p.of(58.9089, 9.2764, 90);
    private final Pose controlPose = p.of(27.0257, 35.1975, 90);
    private final Pose park = p.of(13, 96.9354, 90);

    private Path path1() {
        return line(startPose, controlPose).linear(startPose, controlPose);
    }

    private Path path2() {
        return line(controlPose, park).linear(controlPose, park);
    }

    /**
     * Base Ivy Command implementation with sensible defaults.
     */
    private abstract static class BaseCommand implements Command {
        @Override public Set<Object> requirements() { return Collections.emptySet(); }
        @Override public int priority() { return 0; }
        @Override public InterruptedBehavior interruptedBehavior() { return InterruptedBehavior.END; }
        @Override public ConflictBehavior conflictBehavior() { return ConflictBehavior.CANCEL; }
        @Override public BlockedBehavior blockedBehavior() { return BlockedBehavior.CANCEL; }
        @Override public void start() {}
        @Override public void execute() {}
        @Override public boolean done() { return true; }
        @Override public void end(EndCondition endCondition) {}
    }

    /**
     * Command to spin up the shooter flywheel to target velocity and wait until within tolerance.
     */
    private Command spinUpShooter(double targetRPM) {
        return new BaseCommand() {
            @Override
            public void start() {
                leftshooterPID.setTarget(targetRPM);
                rightshooterPID.setTarget(targetRPM);
            }

            @Override
            public boolean done() {
                double avgVelo = (leftshooterPID.velocity() + rightshooterPID.velocity()) / 2.0;
                return Math.abs(targetRPM - avgVelo) < 50;
            }
        };
    }

    /**
     * Command to run intake and transfer to feed elements into the shooter for a specified duration.
     */
    private Command shootElements(double durationSeconds) {
        return new BaseCommand() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public void start() {
                timer.reset();
                intake.setPower(0.6);
                transfer.setPower(0.4);
            }

            @Override
            public boolean done() {
                return timer.seconds() >= durationSeconds;
            }

            @Override
            public void end(EndCondition endCondition) {
                intake.setPower(0.0);
                transfer.setPower(0.0);
            }
        };
    }

    /**
     * Command to stop shooter flywheels and intake/transfer.
     */
    private Command stopShooter() {
        return new BaseCommand() {
            @Override
            public void start() {
                leftshooterPID.stop();
                rightshooterPID.stop();
                intake.setPower(0.0);
                transfer.setPower(0.0);
            }

            @Override
            public boolean done() {
                return true;
            }
        };
    }

    private Command autoRoutine() {
        return sequential(
                // 1. Spin up shooter flywheel to target velocity
                spinUpShooter(TARGET_SHOOT_VELOCITY),
                // 2. Feed and launch elements using intake and transfer for 5 seconds
                shootElements(5.0),
                // 3. Turn off shooter flywheels and intake/transfer motors
                stopShooter(),
                // 4. Follow path to park
                follow(follower, path1()),
                follow(follower, path2())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();

        // Drivetrain & PedroPathing Follower setup
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        // Mechanisms initialization
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        leftshooterPID = new VelocityPID(hardwareMap, "leftshoot", DcMotorEx.Direction.REVERSE,
                tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);
        rightshooterPID = new VelocityPID(hardwareMap, "rightshoot", DcMotorEx.Direction.FORWARD,
                tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            // Continuously update shooter Velocity PIDs
            leftshooterPID.update();
            rightshooterPID.update();

            double shooterAvg = (leftshooterPID.velocity() + rightshooterPID.velocity()) / 2.0;

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());
            telemetry.addData("Flywheel Velocity Avg", "%.1f", shooterAvg);
            telemetry.addData("Left Shooter Velocity", "%.1f", leftshooterPID.velocity());
            telemetry.addData("Right Shooter Velocity", "%.1f", rightshooterPID.velocity());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
