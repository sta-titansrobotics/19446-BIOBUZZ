package biobuzz.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * basic dcmotor pid control for velocity targets
 * usage:
 *
 * private VelocityPID velocitypid;
 * velocitypid = new VelocityPID(*insert params here*);
 * velocitypid.function(params);
 */
public class VelocityPID {
    private DcMotor motor;

    // PID State Variables
    double preverr = 0;
    double previous = 0;
    double velocity = 0;
    private double deriv = 0;
    private double integ = 0;
    private double target = 0;


    private ElapsedTime timer = new ElapsedTime();
    private double kp, ki, kd;

    //clamping function
    public double clamp(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    //configuring motor to control
    public VelocityPID(HardwareMap hardwareMap, String motor_name, DcMotorSimple.Direction direction, double kp, double ki, double kd){
        motor = hardwareMap.get(DcMotor.class, motor_name);
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

        //reset values
        timer.reset();
    }

    //sets pid target
    public void setTarget(double targetrpm) {
        this.target = targetrpm;
        this.deriv = 0;
        this.integ = 0;
        this.preverr = 0;
        this.previous = motor.getCurrentPosition();
        this.timer.reset();
    }

    //stops the pid loop
    public void stop() {
        this.target = 0;
        this.deriv = 0;
        this.integ = 0;
        motor.setPower(0);
        this.timer.reset();
    }

    //actual update loop
    public void update() {
        double current = motor.getCurrentPosition();
        double deltaTime = timer.seconds();

        FtcDashboard dashboard = FtcDashboard.getInstance();

        if (deltaTime != 0)
            velocity = (current-previous)/deltaTime;

        double error = target - velocity;

        //I and D calculations
        integ += error*deltaTime;
        if (deltaTime != 0) {
            deriv = (error - preverr) / deltaTime;
        } else {
            deriv = 0;
        }

        //power calculation
        double power = (kp * error) + (ki * integ) + (kd * deriv);
        motor.setPower(clamp(power, -1, 1));

        //telemetry
        dashboard.getTelemetry().addData("power", motor.getPower());
        dashboard.getTelemetry().addData("position", current);
        dashboard.getTelemetry().addData("target", target);
        dashboard.getTelemetry().addData("error", error);
        dashboard.getTelemetry().addData("integral", integ);
        dashboard.getTelemetry().addData("derivative", deriv);
        dashboard.getTelemetry().update();

        timer.reset();
        preverr = error;
        previous = current;
    }

    public boolean withinRange(double toleranceTicks) {
        // Check if current position is within the defined tolerance range of the target
        return Math.abs(target - velocity) <= toleranceTicks;
    }

    public double velocity(){
        return velocity;
    }
}
