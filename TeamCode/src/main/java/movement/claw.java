package movement;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
//import movement.tuning;

@TeleOp
public class claw extends LinearOpMode {

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

    double prevtime;

    static double dir;
    static double mag;
    static double pi = Math.PI;

    int lexttarg;
    int rexttarg;
    int lexttargfine;
    int rexttargfine;
    double rexterr;
    double lexterr;
    double Lextpower;
    double Rextpower;
    double rextpreverr;
    double lextpreverr;

    double rottarg;
    double roterr;
    double rotpower;
    double rotpreverr;
    double totroterr;
    double rotinteg;

    // SHOOTER PID VARIABLES
    double shooterTarg;
    double shooterErr;
    double shooterPower;
    double shooterPrevError;
    double shooterDeltaErr;

    double shooterKp;
    double shooterKd;

    double skd = 0.0002; // increase if not getting steady results????
    double skp = 0.08; // increase if you overshoot


    double torquetarg;

    double rot;

    double offset = 0;
    double oroffset = 0;
    double imureset = 0;
    double sqrt2 = Math.sqrt(2);

    double wristYaw;

    int ypod;
    int xpod;
    double ycord;
    double xcord;

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    Orientation lastAngles1 = new Orientation();
    double angle;
    double angle1;
    boolean autorot = false;
    boolean targrot = false;
    boolean aprilrot = false;
    double contangle;

    double velo;
    double prevpos;
    double prevtime1;

    double rotvelo;
    double prevpos1;

    double shootp;
    boolean holdrot;
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

    int cnt = 0;
    double leftlim = 0.65;
    double rightlim = 0.65;
    boolean hitlim = false;

    double stickx;
    double sticky;

    double cntfreeze;



    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    @Override
    public void runOpMode() {

        // ----------------------Set Up------------------------------------------------
        // Moving
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "right"); // Expansion hub
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "left"); // Expansion hub

        CRServo drop = hardwareMap.get(CRServo.class, "drop");

        TouchSensor rxlim = hardwareMap.get(TouchSensor.class, "rxlim");
        TouchSensor lxlim = hardwareMap.get(TouchSensor.class, "lxlim");
        TouchSensor rylim = hardwareMap.get(TouchSensor.class, "rylim");
        TouchSensor lylim = hardwareMap.get(TouchSensor.class, "lylim");

        left.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * prevtime = getRuntime();
         * if (getRuntime() - prevtime > 5000)
         */


        //initAprilTag();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested())
            return;

        while (opModeIsActive()) {

            stickx = -gamepad1.left_stick_x;
            sticky = gamepad1.left_stick_y;

            if (rylim.isPressed() && sticky > 0){
                sticky = 0;
            }
            if (lylim.isPressed() && sticky < 0){
                sticky = 0;
            }
            if (rxlim.isPressed() && stickx > 0){
                stickx = 0;
            }
            if (lxlim.isPressed() && stickx < 0){
                stickx = 0;
            }


            left.setPower(0.5 * (sticky + stickx));
            right.setPower(0.5 * (sticky - stickx));

            drop.setPower(gamepad1.right_stick_y);

            telemetry.addData("left amps", left.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right amps", right.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lx", lxlim.isPressed());
            telemetry.addData("rx", rxlim.isPressed());
            telemetry.addData("ly", lylim.isPressed());
            telemetry.addData("ry", rylim.isPressed());
            telemetry.addData("leftpower", left.getPower());
            telemetry.addData("rightpower", right.getPower());
            telemetry.update();
            if (gamepad1.right_stick_button) {
                cnt = 0;
                while (cnt < 2000) {
                    while (cnt < 400) {
                        cnt++;
                        sleep(20);
                        drop.setPower(1);
                        telemetry.addData("cnt", cnt);
                        telemetry.update();
                    }
                    while (cnt < 760) {
                        cnt++;
                        sleep(20);
                        drop.setPower(-1);
                        telemetry.addData("cnt", cnt);
                        telemetry.update();
                        prevtime = getRuntime();
                    }
                    drop.setPower(0);
                    cnt++;
                }
            }
        }
    }
}