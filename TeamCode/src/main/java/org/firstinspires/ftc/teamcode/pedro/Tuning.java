package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.tuning.autotune.Procedure;
import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

public class Tuning {

    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    public static Procedure foresightTuner() {
        return new ForesightTuner(Constants::createLocalizer, Constants::createDrivetrain);
    }

    public static Procedure tests() {
        return new Tests(Constants::createDrivetrain, Constants::createLocalizer, Constants::createAlgorithm);
    }
}
