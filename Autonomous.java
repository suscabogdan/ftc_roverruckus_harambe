package org.firstinspires.ftc.teamcode.opmodes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.PoseStorage;
import org.firstinspires.ftc.teamcode.opmodes.SimpleHardware;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous(name = "RedLeftFull", group = "Red Auto")
public class RedLeftFull extends LinearOpMode {
    OpenCvCamera webcam;
    RingsDeterminationPipeline pipeline;
    SimpleHardware map = new SimpleHardware();

    @Override
    public void runOpMode()
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        map.init(hardwareMap);
        resetServos();


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingsDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }
        });

        waitForStart();

        while (opModeIsActive())
        {
            final int robotRadius = 9; // inches

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);

            if (pipeline.position == RingsDeterminationPipeline.RingPosition.NONE) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseA(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.ONE) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseB(drive);
                sleep(30000);
            } else if (pipeline.position == RingsDeterminationPipeline.RingPosition.FOUR) {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseC(drive);
                sleep(30000);
            } else {
                webcam.stopStreaming();
                webcam.stopRecordingPipeline();
                caseA(drive);
                sleep(30000);
            }

        }
    }


    public static class RingsDeterminationPipeline extends OpenCvPipeline
    {
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(930,180);

        static final int REGION_WIDTH = 160;
        static final int REGION_HEIGHT = 170;

        final int FOUR_RING_THRESHOLD = 150;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile RingPosition position = RingPosition.FOUR;

        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);
            avg1 = (int) Core.mean(region1_Cb).val[0];
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);

            position = RingPosition.FOUR;
            if(avg1 > FOUR_RING_THRESHOLD) {
                position = RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = RingPosition.ONE;
            } else {
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }

    Pose2d startPose = new Pose2d(-63.0, -24.3, Math.toRadians(0.0));
    private void caseA(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-3.0, -18.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(12.0, -56.0))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-36.0, -50.0))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineTo(new Vector2d(24.0, -60.0))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(10.0, -25.0))
                .build();

        rotateTurret(0.13);
        setShooterPower(1, 0.28);
        drive.followTrajectory(traj1);
        sleep(3300);
        flicker();
        sleep(500);
        flicker();
        sleep(500);
        flicker();
        sleep(300);
        setShooterPower(0, 0.28);
        drive.followTrajectory(traj2);
        leaveFirstWobble();
        placeWobbleGoal();
        sleep(200);
        ungrabWobbleGoal();
        drive.followTrajectory(traj3);
        grabWobbleGoal();
        sleep(500);
        drive.followTrajectory(traj4);
        ungrabWobbleGoal();
        sleep(500);
        drive.followTrajectory(traj5);

        sleep(500);
//        intakeRings(0);

        map.wobbleSideServo.setPosition(0.0);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void caseB(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-3.0, -18.0))
                .addDisplacementMarker(7, () -> {
                    // This marker runs 7 inch into the trajectory
//                    placeWobbleGoal();
                })
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(36.0, -26.5))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(-23.0, -36.0, Math.toRadians(180.0)))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(-27.0, -36.0, Math.toRadians(0.0)))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineTo(new Vector2d(-39.0, -47.0))
                .build();
        Trajectory traj6 = drive.trajectoryBuilder(traj5.end())
                .lineTo(new Vector2d(-1.0, -36.0))
                .build();
        Trajectory traj7 = drive.trajectoryBuilder(traj6.end())
                .lineToSplineHeading(new Pose2d(28.0, -23.5, Math.toRadians(90)))
                .build();
        Trajectory traj8 = drive.trajectoryBuilder(traj7.end())
                .lineToSplineHeading(new Pose2d(28.0, -14.0, Math.toRadians(0)))
                .build();
        Trajectory traj9 = drive.trajectoryBuilder(traj8.end())
                .lineToSplineHeading(new Pose2d(5.0, -14.0, Math.toRadians(0)))
                .build();

        rotateTurret(0.13);
        setShooterPower(1, 0.28);
        drive.followTrajectory(traj1);
        sleep(3200);
        flicker();
        sleep(400);
        flicker();
        sleep(500);
        flicker();
        sleep(300);
        setShooterPower(0, 0.28);
        drive.followTrajectory(traj2);
        leaveFirstWobble();
        placeWobbleGoal();
        sleep(200);
        ungrabWobbleGoal();
        intakeRings(1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);
        grabWobbleGoal();
        sleep(200);
        rotateTurret(0.17);
        setShooterPower(1, 0.286);
        drive.followTrajectory(traj6);
        intakeRings(0);
        sleep(1000);
        flicker();
        sleep(200);
        setShooterPower(0, 0.28);
        drive.followTrajectory(traj7);
        sleep(500);
        ungrabWobbleGoal();
        sleep(1000);
        drive.followTrajectory(traj8);
        drive.followTrajectory(traj9);

        map.wobbleSideServo.setPosition(0.0);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void caseC(SampleMecanumDrive drive) {
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(-22.0, -41))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineTo(new Vector2d(-18.0, -41),
                new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                        )
                ),
                new ProfileAccelerationConstraint(20))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineTo(new Vector2d(-5, -41),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(DriveConstants.MAX_ANG_VEL),
                                        new MecanumVelocityConstraint(20, DriveConstants.TRACK_WIDTH)
                                )
                        ),
                        new ProfileAccelerationConstraint(20))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .lineToSplineHeading(new Pose2d(48.0, -45.0, Math.toRadians(30)))
                .build(); //duce wobble
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .lineToSplineHeading(new Pose2d(5, -35.0, Math.toRadians(0)))
                .build();

        rotateTurret(0.194);
        setShooterPower(1, 0.27);
        drive.followTrajectory(traj1);
        sleep(1500);
        flicker();
        sleep(700);
        flicker();
        sleep(700);
        flicker();
        sleep(400);
        intakeRings(1);
        drive.followTrajectory(traj2);
        sleep(4000);
        flicker();
        sleep(200);
        setShooterPower(1, 0.28);
        drive.followTrajectory(traj3);
        sleep(1000);
        intakeRings(0);
        rotateTurret(0.2);
        sleep(400);
        flicker();
        sleep(500);
        flicker();
        sleep(500);
        flicker();
        sleep(200);
        setShooterPower(0, 0.28);
        drive.followTrajectory(traj4);
        leaveFirstWobble();
        sleep(200);
        drive.followTrajectory(traj5);

        PoseStorage.currentPose = drive.getPoseEstimate();
    }

    private void intakeRings(double power) {
        map.intakeMotor.setPower(power);
        map.intakeMotor2.setPower(power);
    }

    private void returnWobbleArm() {
        map.wobbleMotor.setTargetPosition(75);
        map.wobbleMotor.set(0);
        map.wobbleMotor.setPositionTolerance(20);

        while (!map.wobbleMotor.atTargetPosition()) {
            map.wobbleMotor.set(0.25);
        }

        map.wobbleMotor.stopMotor();
        map.wobbleMotor.set(-0.16);
    }

    private void placeWobbleGoal() {
        map.wobbleMotor.setTargetPosition(118);
        map.wobbleMotor.set(0);
        map.wobbleMotor.setPositionTolerance(20);

        while (!map.wobbleMotor.atTargetPosition()) {
//            t.addData("Motor pos", arm.getCurrentPosition());
//            t.update();
            map.wobbleMotor.set(0.15);
        }

        map.wobbleMotor.stopMotor();
    }

    private void grabWobbleGoal(){
        map.wobbleClawLeft.setPosition(0.48);
        map.wobbleClawRight.setPosition(0.48);
        sleep(200);
    }

    private void ungrabWobbleGoal(){
        map.wobbleClawLeft.setPosition(0.0);
        map.wobbleClawRight.setPosition(0.0);
        sleep(200);
    }

    private void setShooterPower(double power, double servoPosition) {
        map.shooterFrontMotor.setPower(power);
        map.shooterServo.setPosition(servoPosition);
    }

    private void flicker() {
        double feederInit = 0.0, feederPush = 0.15;
        map.feederServo.setPosition(feederPush);
        sleep(400);
        map.feederServo.setPosition(feederInit);
        sleep(200);
    }

    private void blockersUp() {
        map.ringBlockerLeft.setPosition(0.42);
        map.ringBlockerRight.setPosition(0.42);
    }

    private void leaveFirstWobble() {
        map.wobbleSideServo.setPosition(0.32);
    }

    private void rotateTurret(double position) {
        map.turretServo.setPosition(position);
    }

    private void resetServos() {
        map.feederServo.setPosition(0.0);
        map.shooterServo.setPosition(0.0);
        map.turretServo.setPosition(0.21);
        map.ringBlockerRight.setPosition(0.00);
        map.ringBlockerLeft.setPosition(0.0);
        map.wobbleSideServo.setPosition(0.0);
        map.wobbleClawLeft.setPosition(0.0);
        map.wobbleClawRight.setPosition(0.0);
    }
}
