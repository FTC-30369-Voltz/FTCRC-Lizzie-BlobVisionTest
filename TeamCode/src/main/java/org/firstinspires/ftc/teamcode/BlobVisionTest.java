package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.seattlesolvers.solverslib.drivebase.MecanumDrive;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

@TeleOp(name = "Blob Vision Test")
public class BlobVisionTest extends LinearOpMode {

    private Motor frontLeft;
    private Motor backLeft;
    private Motor frontRight;
    private Motor backRight;

    public Limelight3A camera;

    public GamepadEx gamepadEx;

    public double forward = 0.0;
    public double turn = 0.0;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        frontLeft = new Motor(hardwareMap, "frontLeft");
        backLeft = new Motor(hardwareMap, "backLeft");
        frontRight = new Motor(hardwareMap, "frontRight");
        backRight = new Motor(hardwareMap, "backRight");
        camera = hardwareMap.get(Limelight3A.class, "limelight");
        gamepadEx = new GamepadEx(gamepad1);

        MecanumDrive mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            camera.pipelineSwitch(1);
            camera.start();
            telemetry.addLine("Camera started!");
            gamepadEx.gamepad.rumbleBlips(2);
            telemetry.update();
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.

                forward = 0.;
                turn = 0.;
                telemetry.update();
                LLResult llResult = camera.getLatestResult();

                if (llResult != null) {
                    if (llResult.isValid()) {
                        gamepadEx.gamepad.rumble(llResult.getTa()/100., llResult.getTa()/100., 100);
                        telemetry.addLine("Camera pipeline satisfied!");
                        telemetry.addData("Blob X Position", llResult.getTx());
                        telemetry.addData("Blob Y Position", llResult.getTy());
                        telemetry.addData("Percentage in View", llResult.getTa());

                        /*
                        Based on: https://docs.limelightvision.io/docs/docs-limelight/pipeline-retro/retro-theory
                        and https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
                        */

                        double nx = ((double) 1 /320) * (llResult.getTx()-319.5);
                        double ny = ((double) 1 /240) * (llResult.getTx()-239.5);
                        double vpw = 2*Math.tan((54.505/2)*(3.14159/180));
                        double vph = 2*Math.tan((42.239/2)*(3.14159/180));
                        double x = vpw/2*nx;
                        double y = vph/2*ny;
                        double ay = Math.atan2(y, (1/(llResult.getTa()/10)));

                        double hyp = 1 / Math.tan(ay);
                        double distance = (hyp/llResult.getTy())*0.0254;
                        telemetry.addData("Distance", Math.abs(distance));
                        telemetry.addData("Botpose", llResult.getBotpose());

                        if (Math.abs(llResult.getTx()) > 2) {
                            turn = -(llResult.getTx()/20);
                        }
                        if (Math.abs(distance) > 0.01) {
                            forward = 0.5;
                        }
                    } else {
                        telemetry.addLine("Camera awaiting pipeline satisfying scenario");
                    }
                    mecanum.driveRobotCentric(0, -forward, turn);
                }
            }
        }
        telemetry.addLine("Camera stopping...");
        telemetry.update();
        camera.stop();
    }
}
