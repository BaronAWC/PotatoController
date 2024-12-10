package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();
        Pose2d initialPose = new Pose2d(-13, -62, Math.toRadians(180));
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-13, -62, Math.toRadians(180)))


                

                .strafeTo(new Vector2d(-15, -59))  // -11-4=-15 for left, -63+4=-59 for up
                .splineToSplineHeading(new Pose2d(-57, -61, Math.toRadians(220)), Math.toRadians(220))
                //atbucket
                .waitSeconds(5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.45, -25.45, Math.toRadians(180.00)), Math.toRadians(90))
                //atpisitiontogetfirstpiece
                .waitSeconds(4)
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(-62, -56), Math.toRadians(240))
                //backatbucket
                .waitSeconds(5)
                .setReversed(true)
                .strafeToSplineHeading(new Vector2d(-36.45, -25.45), Math.toRadians(180))
                .waitSeconds(4)
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(-62, -56), Math.toRadians(240))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-36.45, -25.45, Math.toRadians(180.00)), Math.toRadians(90))
                .setReversed(false)
                .strafeToSplineHeading(new Vector2d(-62, -56), Math.toRadians(240))
                .strafeToLinearHeading(new Vector2d(-22,7), Math.toRadians(180.00))







                /*        // Strafe diagonally (4 up, 4 left)
                .strafeTo(new Vector2d(-15, -59))  // -11-4=-15 for left, -63+4=-59 for up
                // Spline to final position
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(225)), Math.toRadians(180))

                // Backward spline to second position
                .setReversed(true)  // Set to move backwards
                .splineTo(new Vector2d(-35, -46), Math.toRadians(90))  // First curve point, moving up
                .strafeTo(new Vector2d(-33,-26))

                .splineTo(new Vector2d(-33, -20), Math.toRadians(180)) // Final position

*/

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}