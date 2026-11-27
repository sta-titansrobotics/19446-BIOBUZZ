package pedro_example;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.conditional;

import com.pedropathing.api.Paths;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Collections;
import java.util.HashSet;
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
    private Limelight3A limelight3A;

    // Target velocity setting for flywheel
    private static final double TARGET_SHOOT_VELOCITY = 740;

    // Limelight
    private double avgTy = 0;
    private int detectedTagCount = 0;
    private double tyBefore = 0;
    private boolean hiveTipped = false;
    private static final double TIP_TY_DROP = 5.0; // degrees — MEASURE THIS on the bench
    private static final int[] TARGET_TAG_IDS = {30, 31, 32, 33, 34, 35, 36, 37};


    // Poses
    private final Pose startPose = p.of(58.9089, 9.2764, 180);
    private final Pose HiveTip2 = p.of(60, 130, 0);
    private final Pose intakePose = p.of(11, 9.2764, 180);

    private final Pose controlPose = p.of(30.19, 22.08, -180);


    private final Pose park = p.of(13, 96, -180);

    private Path path1() {
        return line(startPose, intakePose).linear(startPose, intakePose);
    }

    private Path path2() {
        return line(intakePose, startPose).linear(intakePose, startPose);
    }

    private Path path3() {
        return line(startPose, controlPose).linear(startPose, controlPose);
    }

    private Path path4() {
        return line(controlPose, park).linear(controlPose, park);
    }

    private Path path5() {
        return line(startPose, HiveTip2).linear(startPose, HiveTip2);
    }

    private Path path6() {
        return line(HiveTip2, park).linear(HiveTip2, park);
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

    private void updateLimelight() {
        if (limelight3A == null) return;
        LLResult result = limelight3A.getLatestResult();
        if (result == null || !result.isValid()) {
            detectedTagCount = 0;
            avgTy = 0;
            return;
        }
        double sumTy = 0;
        Set<Integer> seen = new HashSet<>();
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            int id = f.getFiducialId();
            boolean target = false;
            for (int t : TARGET_TAG_IDS) if (t == id) target = true;
            if (target && seen.add(id)) sumTy += f.getTargetYDegrees();
        }
        detectedTagCount = seen.size();
        avgTy = detectedTagCount > 0 ? sumTy / detectedTagCount : 0;
    }

    private Command recordTyBefore() {
        return new BaseCommand() {
            @Override public void start() {
                if (detectedTagCount > 0) tyBefore = avgTy;
            }
            @Override public boolean done() { return true; }
        };
    }

    private Command checkTip() {
        return new BaseCommand() {
            @Override public void start() {
                // No tags visible -> default to "not tipped" so it shoots again.
                hiveTipped = detectedTagCount == 0;
            }
            @Override public boolean done() { return true; }
        };
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
     * Intake that runs until it is cancelled. Used inside a deadline() group
     * with a path: it starts when the path starts, and its end() stops the
     * motors when the path finishes. done() must return false — the base
     * class returns true, which would end the intake on the first tick.
     */
    Command intakeElements() {
        return new BaseCommand() {
            @Override
            public void start() {
                intake.setPower(1);
                transfer.setPower(1);
            }

            @Override
            public boolean done() {
                return false;
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

                recordTyBefore(),
                spinUpShooter(TARGET_SHOOT_VELOCITY),
                shootElements(5.0),

                waitMs(1000),

                checkTip(),

                stopShooter(),
                // 4. Drive path1 while intaking: the path is the deadline, so the
                // intake runs for exactly as long as the drive takes, then stops.
                deadline(follow(follower, path1()), intakeElements()),

                follow(follower, path2()),



                conditional(
                        () -> hiveTipped,
                        sequential(                          // true: it tipped, skip volley 2
                                follow(follower, path5()),

                                spinUpShooter(TARGET_SHOOT_VELOCITY),
                                shootElements(5.0),

                                stopShooter(),

                                follow(follower, path6())
                        ),
                        sequential(                          // false: didn't tip, shoot volley 2
                                spinUpShooter(TARGET_SHOOT_VELOCITY),
                                shootElements(5.0),
                                stopShooter(),
                                follow(follower, path3()),
                                follow(follower, path4())
                        )
                )
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

        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        limelight3A.start();

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
            updateLimelight();
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
            telemetry.addData("Tags", detectedTagCount);
            telemetry.addData("Avg TY", "%.2f", avgTy);
            telemetry.addData("TY before", "%.2f", tyBefore);
            telemetry.addData("HIVE tipped", hiveTipped);

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }
}
