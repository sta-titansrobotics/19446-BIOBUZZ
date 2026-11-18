package biobuzz;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import biobuzz.subsystems.MotorPID;
import biobuzz.subsystems.VelocityPID;
import biobuzz.tuning;

@TeleOp
public class coords_auto extends OpMode {

    int button2X = 0;
    int button2A = 0;
    int button2B = 0;
    int button2Y = 0;
    int buttonA = 0;
    int buttonB = 0;
    int buttonX = 4;
    int buttonY = 0;

    boolean butAcheck = false;
    boolean butYcheck = false;
    boolean butXcheck = false;
    boolean butBcheck = false;
    boolean but2Acheck = false;
    boolean but2Ycheck = false;
    boolean but2Xcheck = false;
    boolean but2Bcheck = false;

    static double dir;
    static double mag;
    static double pi = Math.PI;

    double rottarg;
    double roterr;
    double rotpower;
    double rotpreverr;
    double totroterr;
    double rotinteg;

    double rot;

    double offset = 0;
    double oroffset = 0;
    double imureset = 0;
    double sqrt2 = Math.sqrt(2);

    //BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation lastAngles1 = new Orientation();
    double angle;
    double angle1;

    double apriltuner = 0.7;

    List<AprilTagDetection> detections;

    private Limelight3A limelight3A;
    private LLResult llResult;
    private double distance;
    int id;
    boolean shootenabled = false;
    boolean pidActiveLastLoop = false;
    double lastTime = 0;

    boolean patternunlock = false;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    RevBlinkinLedDriver underglow;
    RevBlinkinLedDriver aimassist;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    RevBlinkinLedDriver.BlinkinPattern prevpattern;

    private DcMotor FL;
    private DcMotor BL;
    private DcMotor FR;
    private DcMotor BR;

    private DcMotor intake;
    private DcMotor rightShoot;
    private DcMotor leftShoot;
    private DcMotor transfer;

    VelocityPID leftshooterPID;
    VelocityPID rightshooterPID;

    FtcDashboard dashboard;

    @Override
    public void init() {
        // ----------------------Set Up------------------------------------------------
        // Moving
        FL = hardwareMap.get(DcMotor.class, "FL");
        BL = hardwareMap.get(DcMotor.class, "BL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BR = hardwareMap.get(DcMotor.class, "BR");

        intake = hardwareMap.get(DcMotor.class, "intake");
        rightShoot = hardwareMap.get(DcMotor.class, "rightshoot");
        leftShoot = hardwareMap.get(DcMotor.class, "leftshoot");
        transfer = hardwareMap.get(DcMotor.class, "transfer");


        //limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight3A.pipelineSwitch(0);
        //limelight3A.start();


        //---------CHANGE---------------
        /*
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class,"imu");

        imu.initialize(parameters);

         */

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */

        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);

        leftshooterPID = new VelocityPID(hardwareMap, "leftshoot", DcMotorSimple.Direction.REVERSE, tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);
        rightshooterPID = new VelocityPID(hardwareMap, "rightshoot", DcMotorSimple.Direction.FORWARD, tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);

        //initAprilTag();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {

        if (gamepad1.a){
            rightshooterPID.setTarget(1000);
            leftshooterPID.setTarget(1000);
        } else {
            rightshooterPID.setTarget(0);
            leftshooterPID.setTarget(0);
        }

        // ------------------DRIVE TRAIN---------------------------------
        //will cause the offset to be set back to 0

        //offset = (fieldangle() - imureset);
        //oroffset = (getAngle()%360) - imureset;

        //imu increases when turning left and decreases when turning right

        //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

        //the dir variable is the variable that determines where we want to be on the sine wave

        //if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
        //    imureset = fieldangle();

        dir = Math.atan2(-gamepad1.right_stick_y, gamepad1.right_stick_x) - offset;
        mag = Math.sqrt(Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2));
        mag *= sqrt2;
        if (mag > sqrt2)
            mag = sqrt2;


        //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
        if (gamepad1.right_trigger>0.1)
            rot *= 0.5;
        if (gamepad1.right_trigger>0.1)
            mag *= 0.5;


        if (gamepad1.right_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || rot != 0) {
            FR.setPower((Math.sin(dir - (pi / 4)) * mag) - rot);
            FL.setPower((Math.sin(dir + (pi / 4)) * mag) + rot);
            BR.setPower((Math.sin(dir + (pi / 4)) * mag) - rot);
            BL.setPower((Math.sin(dir - (pi / 4)) * mag) + rot);
        } else {
            FL.setPower(0);
            BL.setPower(0);
            FR.setPower(0);
            BR.setPower(0);
        }

        telemetry.addLine("Drivetrain");
        telemetry.addLine("");
        telemetry.addData("FR Power", FR.getPower());
        telemetry.addData("FL Power", FL.getPower());
        telemetry.addData("BR Power", BR.getPower());
        telemetry.addData("BL Power", BL.getPower());


        telemetry.addData("right flywheel spin", rightshooterPID.velocity());
        telemetry.addData("left flywheel spin", leftshooterPID.velocity());




        telemetry.addLine("");
        telemetry.addLine("<------------------->");
        telemetry.addLine("");

        //telemetry.addData("", );

        //-----------------apriltagsssssssss------------------------------------

        telemetry.update();
        dashboard.getTelemetry().update();
        //updateLimelight();
        leftshooterPID.update();
        rightshooterPID.update();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}

    /*
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle += deltaAngle;

        lastAngles = angles;


        return angle;
    }

    private double fieldangle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double deltaAngle = angles.firstAngle - lastAngles1.firstAngle;

        angle1 += deltaAngle;

        lastAngles1 = angles;

        return angle1;
    }
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

     */

    /*
    private void updateLimelight() {
        limelight3A.updateRobotOrientation(getAngle());

        llResult = limelight3A.getLatestResult();

        if (llResult != null && llResult.isValid()) {
            // Iterate through all detected fiducials (AprilTags)
            for (com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult fiducial : llResult.getFiducialResults()) {
                id = fiducial.getFiducialId();

                // Only update distance if it's Tag 20 or Tag 24
                if (id == 20 || id == 24) {
                    // The Limelight SDK gives us target area as a normalized value (0-1),
                    // but our formula needs it as a percentage (0-100).
                    distance = getDistanceFromTag(fiducial.getTargetArea() * 100.0);
                    telemetry.addData("Target Area", fiducial.getTargetArea());


                    return; // Exit once we found our target tag
                }
            }
        }
        // If no valid tag is found, reset distance to 0 to avoid using stale values.
        distance = 0;
    }

    public double getDistanceFromTag(double ta){
        // This equation is derived from a power regression of distance vs. target area
        // from a table of observed values.
        return 157.6 * Math.pow(ta, -0.584);
    }
    public void lightingloop(){
        boolean patternlock = false;

        if (Math.abs(roterr) < 1.5) {
            patternlock = true;
        }

        if (patternlock)
            aimassist.setPattern(pattern);
    }

     */
}