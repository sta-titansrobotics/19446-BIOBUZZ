package biobuzz;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp
public class LimelightDistance extends OpMode {

    private Limelight3A limelight3A;
    private GoBildaPinpointDriver pinpoint;
    private double distance;

    @Override
    public void init(){
        limelight3A = hardwareMap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(0);
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    @Override
    public void start(){
        limelight3A.start();
    }

    @Override
    public void loop(){
        pinpoint.update();
        // updateRobotOrientation takes DEGREES; Pinpoint gives degrees here.
        limelight3A.updateRobotOrientation(pinpoint.getHeading(AngleUnit.DEGREES));

        LLResult llResult = limelight3A.getLatestResult();
        if(llResult != null && llResult.isValid()){
            Pose3D botPose = llResult.getBotpose_MT2();
            distance = getDistanceFromTag(llResult.getTa());
            telemetry.addData("Distance", distance);
            telemetry.addData("Target x", llResult.getTx());
            telemetry.addData("Target Area", llResult.getTa());
            telemetry.addData("Botpose", botPose.toString());
        } else {
            telemetry.addLine("NO TARGET");
        }
        telemetry.update();
    }

    public double getDistanceFromTag(double ta){
        // ta from getTa() is already a percentage (0-100), which is what
        // this formula expects. (The per-fiducial getTargetArea() is 0-1,
        // which is why coords_auto multiplies by 100 — different getter.)
        return 157.6 * Math.pow(ta, -0.584);
    }
}
