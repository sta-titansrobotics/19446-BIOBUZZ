package biobuzz;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import biobuzz.subsystems.CRservoPID;
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

    double rot;

    double offset = 0;
    double oroffset = 0;
    double imureset = 0;
    double sqrt2 = Math.sqrt(2);

    //BNO055IMU imu;
    //Orientation lastAngles = new Orientation();
    //Orientation lastAngles1 = new Orientation();
    double lastAngles;
    double lastAngles1;
    double lastAngles2;
    double angle;
    double angle1;
    double angle2;

    double apriltuner = 0.7;

    List<AprilTagDetection> detections;

    private Limelight3A limelight3A;
    private LLResult llResult;
    private double distance;
    int id;
    double lastTime = 0;

    boolean patternunlock = false;


    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    private DcMotorEx FL;
    private DcMotorEx BL;
    private DcMotorEx FR;
    private DcMotorEx BR;

    private DcMotorEx intake;
    private DcMotorEx rightShoot;
    private DcMotorEx leftShoot;
    private DcMotorEx transfer;

    private CRServo leftExt;
    private CRServo rightExt;
    private CRServo righthood;
    private CRServo lefthood;
    

    VelocityPID leftshooterPID;
    VelocityPID rightshooterPID;
    CRservoPID righthoodPID;
    CRservoPID lefthoodPID;

    boolean readyToShoot;
    double shooteravg;
    double hood_position;

    private GoBildaPinpointDriver pinpoint;


    FtcDashboard dashboard;

    @Override
    public void init() {
        // ----------------------Set Up------------------------------------------------
        // Moving
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightshoot");
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftshoot");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        leftExt = hardwareMap.get(CRServo.class, "intake1");
        rightExt = hardwareMap.get(CRServo.class, "intake2");
        lefthood = hardwareMap.get(CRServo.class, "lefthood");
        righthood = hardwareMap.get(CRServo.class, "righthood");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

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

        BR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        BL.setDirection(DcMotorEx.Direction.REVERSE);
        FL.setDirection(DcMotorEx.Direction.REVERSE);

        intake.setDirection(DcMotorEx.Direction.REVERSE);

        leftExt.setDirection(DcMotorEx.Direction.REVERSE);
        lefthood.setDirection(DcMotorEx.Direction.REVERSE);

        leftshooterPID = new VelocityPID(hardwareMap, "leftshoot", DcMotorEx.Direction.REVERSE, tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);
        rightshooterPID = new VelocityPID(hardwareMap, "rightshoot", DcMotorEx.Direction.FORWARD, tuning.shootkp, tuning.shootki, tuning.shootkd, tuning.shootkf, tuning.shootmax, tuning.shootmin);

        lefthoodPID = new CRservoPID(hardwareMap, "lefthood", DcMotorEx.Direction.FORWARD, tuning.hoodkp, tuning.hoodki, tuning.hoodkd);
        righthoodPID = new CRservoPID(hardwareMap, "righthood", DcMotorEx.Direction.FORWARD, tuning.hoodkp, tuning.hoodki, tuning.hoodkd);


        //initAprilTag();
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void loop() {

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        shooteravg = (leftshooterPID.velocity()+rightshooterPID.velocity())/2;
        hood_position = hoodangle();
        
        if (gamepad2.a){
            rightshooterPID.setTarget(tuning.targetvelo);
            leftshooterPID.setTarget(tuning.targetvelo);
        } else {
            rightshooterPID.setTarget(0);
            leftshooterPID.setTarget(0);
        }

        if (Math.abs(tuning.targetvelo-shooteravg)<150){
            readyToShoot = true;
        } else {
            readyToShoot = false;
        }
        
        if (gamepad1.right_trigger>0.1){
            rightExt.setPower(gamepad1.right_trigger);
            leftExt.setPower(gamepad1.right_trigger);
        }


        if (gamepad2.right_trigger>0.1) {
            if (readyToShoot){
                intake.setPower(gamepad2.right_trigger);
                transfer.setPower(gamepad2.right_trigger);
            } else {
                intake.setPower(gamepad2.right_trigger);
                transfer.setPower(-gamepad2.right_trigger);
            }
        } else if (gamepad2.left_trigger>0.1){
            intake.setPower(gamepad2.left_trigger);
            transfer.setPower(-gamepad2.left_trigger);
        } else if (gamepad2.y){
            intake.setPower(-1);
            transfer.setPower(-1);
        } else {
            intake.setPower(0);
            transfer.setPower(0);
        }



        //use x to switch between the two shooting angles for hood
        //make it compensate for the angle of the robot so its field centric angle of hood
        //make velocity of flywheel constant

        //if no hood preset angles, use left joystick x for angle of hood

        //lefthood.setPower(gamepad2.left_stick_x);
        //righthood.setPower(gamepad2.left_stick_x);




        // ------------------DRIVE TRAIN---------------------------------
        //will cause the offset to be set back to 0

        offset = (fieldangle() - imureset);
        //oroffset = (getAngle()%360) - imureset;

        //imu increases when turning left and decreases when turning right

        //the offset variable tells it how much it has deviated from the original orientation so that it can still move in the correct direction when rotated

        //the dir variable is the variable that determines where we want to be on the sine wave

        //if (gamepad1.left_trigger>0.8&&gamepad1.right_trigger>0.8)
        //    imureset = fieldangle();

        rot = gamepad1.right_stick_x;

        dir = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - offset;
        mag = Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2));
        mag *= sqrt2;
        if (mag > sqrt2)
            mag = sqrt2;


        //CHANGE ROTATION TO BE CONTROLLED BY GAMEPAD 2
        if (gamepad1.right_trigger>0.1)
            rot *= 0.5;
        if (gamepad1.right_trigger>0.1)
            mag *= 0.5;


        if (gamepad1.left_stick_y != 0 || gamepad1.right_stick_x != 0 || gamepad1.left_stick_x != 0 || rot != 0) {
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


        telemetry.addData("intake power", intake.getPower());
        telemetry.addData("transfer power", transfer.getPower());
        telemetry.addData("intake velocity", intake.getVelocity());
        telemetry.addData("transfer velocity", transfer.getVelocity());
        telemetry.addData("robot angle", fieldangle());


        telemetry.addLine("");
        telemetry.addLine("<------------------->");
        telemetry.addLine("");

        telemetry.addData("encoder", hood_position);



        //telemetry.addData("", );

        //-----------------apriltagsssssssss------------------------------------

        telemetry.update();
        dashboard.getTelemetry().update();
        //updateLimelight();
        leftshooterPID.update();
        rightshooterPID.update();
        lefthoodPID.update(hood_position);
        righthoodPID.update(hood_position);
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

    private double hoodangle() {
        AnalogInput hoodvolt = hardwareMap.get(AnalogInput.class, "encoder");
        double angles = hoodvolt.getVoltage()/3.3*360;

        double deltaAngle = angles - lastAngles2;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        angle2 += deltaAngle;

        lastAngles2 = angles;

        return angle2;
    }

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

*/
    private double fieldangle() {
        double angles = pinpoint.getHeading(AngleUnit.RADIANS);

        double deltaAngle = angles - lastAngles1;

        angle1 += deltaAngle;

        lastAngles1 = angles;

        return angle1;
    }
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }



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