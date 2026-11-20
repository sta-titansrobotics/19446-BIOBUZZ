package pedro_example;

import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner();
    }

    @Tuner
    public static Procedure tests() {
        return new org.firstinspires.ftc.teamcode.pedro.procedures.Tests(hardwareMap -> new Mecanum(hardwareMap, Constants.drivetrainConfig), null, null);
    }

}
