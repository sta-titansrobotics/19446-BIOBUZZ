package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TestBench extends LinearOpMode {
    // Hardware member variables matching the physical test bench setup
    public DcMotor motor;
    public Servo miniServo;

    private static final double POSITION_CLOSED = 0.2;
    private static final double POSITION_OPEN = 0.8;

    public TouchSensor touchSensor;
    public TouchSensor magneticSwitch;
    public NormalizedColorSensor colorSensor;

    public DistanceSensor distanceSensor;

    // Initialization method called from your OpMode

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware Mapping matching the Driver Station configuration file names
        motor = hardwareMap.get(DcMotor.class, "motor");
        miniServo = hardwareMap.get(Servo.class, "servo");


        // Configured as digital devices for programming flexibility
        touchSensor = hardwareMap.get(TouchSensor.class, "touch_sensor");
        magneticSwitch = hardwareMap.get(TouchSensor.class, "magnetic_sensor");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        miniServo.setPosition(0.5);

        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        colorSensor.setGain(20.0f);

        waitForStart();

        float[] hsvValues = new float[3];

        while (opModeIsActive()){

            double motorPower = -gamepad1.left_stick_y;
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];

            String detectedColor = "";

            // 3. Get Distance measurement (Optimal range for V3 color matching is within 2cm)
            double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);

            if (distanceCm >= 1.0 && distanceCm < 6.0) {
                if (hue >= 40 && hue < 85) {
                    detectedColor = "Yellow";
                } else if (hue > 330 || hue < 30) {
                    detectedColor = "Red";
                } else if (hue >= 180 && hue <= 225){
                    detectedColor = "Blue";
                } else {
                    detectedColor = "Cannot Detect";
                }

            }

            if(gamepad1.a) {
                miniServo.setPosition(POSITION_CLOSED);
            } else if (gamepad1.y) {
                miniServo.setPosition(POSITION_OPEN);
            } else {
                miniServo.setPosition(0.5);
            }

            motor.setPower(motorPower);

            telemetry.addData("Motor Power: ", motor.getPower());
            telemetry.addData("Touch Pressed: ", touchSensor.isPressed());
            telemetry.addData("Magnet Detected: ", magneticSwitch.isPressed());
            telemetry.addData("Detected Object Color", detectedColor);
            telemetry.addData("Distance Closeness (cm)", "%.2f cm", distanceCm);
            telemetry.addData("Servo: ", miniServo.getPosition());
            telemetry.update();
        }

    }

}
