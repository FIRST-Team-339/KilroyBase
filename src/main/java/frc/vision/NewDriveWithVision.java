package frc.vision;

public class NewDriveWithVision {

    /**
     * Aligns to vision targets using a one turn system. For use if already mostly.
     * Will drive up to deposit distance from the targets.
     * 
     * @deprecated commented out until Hardware references are removed.
     *
     * @return
     */
    public boolean driveToTarget() {

        // if (!Hardware.visionInterface.filterPass) {
        //     System.out.println("have no blobs. Cannot continue");
        //     return true;
        // }
        // System.out.println("aligning non arc");
        // double offness = Hardware.visionInterface.getXOffSet();

        // double adjustmentValueRight = 0;
        // double adjustmentValueLeft = 0;

        // System.out.println("camera distance" + Hardware.visionInterface.getDistanceFromTarget());
        // if (Hardware.visionInterface.getDistanceFromTarget() >= 40) {
        //     System.out.println("driving to target");

        //     if (offness < 0) {

        //         adjustmentValueLeft = MIN_MOVE - (Math.abs(offness) * ADJUST_PORP);
        //         adjustmentValueRight = MIN_MOVE + (Math.abs(offness) * ADJUST_PORP);
        //         // Hardware.drive.drive(adjustmentValueLeft,
        //         // adjustmentValueRight);
        //         Hardware.transmission.driveRaw(adjustmentValueLeft, adjustmentValueRight);
        //     }

        //     else
        //     /* if (offness > 0) */
        //     {

        //         adjustmentValueLeft = MIN_MOVE + (Math.abs(offness) * ADJUST_PORP);
        //         adjustmentValueRight = MIN_MOVE - (Math.abs(offness) * ADJUST_PORP);

        //         // Hardware.drive.drive(adjustmentValueLeft,
        //         // adjustmentValueRight);
        //         Hardware.transmission.driveRaw(adjustmentValueLeft, adjustmentValueRight);
        //     }
        //     // else
        //     // {
        //     // Hardware.transmission.driveRaw(.2, .2);
        //     // }
        // } else {
        //     System.out.println("done");
        //     if (Hardware.drive.driveStraightInches(10, .2, 0, true)) {
        //         Hardware.drive.stop();

        //         return true;
        //     }
        // }
        return false;

    }

    boolean secondTurn = false;

    boolean setUp = false;

    double initXoff = 0;

    /**
     * Aligns to vision targets using a two turn system. The first turn centers the
     * robot on the target if starting from a weird angle. The second turn finishes
     * the alignment to the target and drive to them.
     * 
     * @deprecated Commented out until Hardware references are removed.
     *
     * @return
     */
    public boolean driveToTargetArc() {

        // System.out.println("gotta go fast and curvy");
        // // Get x off set angle
        // double offness = Hardware.visionInterface.getXOffSet();

        // // default values
        // double adjustmentValueRight = 0;
        // double adjustmentValueLeft = 0;
        // if (!setUp) {
        //     initXoff = Hardware.visionInterface.getXOffSet();
        //     setUp = true;
        // }

        // if (Hardware.visionInterface.getDistanceFromTarget() >= 40)
        // // TODO check distane with ultra
        // {
        //     if (!secondTurn) {
        //         if (offness < 0) {
        //             if (offness <= (2 * initXoff)) {
        //                 secondTurn = true;
        //             } else {
        //                 Hardware.transmission.driveRaw(MIN_MOVE, MIN_MOVE + .1);
        //             }
        //         } else if (offness > 0) {
        //             if (offness >= (2 * initXoff)) {
        //                 secondTurn = true;
        //             } else {
        //                 Hardware.transmission.driveRaw(MIN_MOVE + .1, MIN_MOVE);
        //             }
        //         }
        //     }
        //     // after second turn
        //     else {

        //         System.out.println("driving to target the second");

        //         if (offness < 0) {
        //             adjustmentValueLeft = MIN_MOVE - (Math.abs(offness) * ADJUST_PORP);
        //             adjustmentValueRight = MIN_MOVE + (Math.abs(offness) * ADJUST_PORP);

        //             Hardware.transmission.driveRaw(adjustmentValueLeft, adjustmentValueRight);
        //         } else
        //         /* if (offness > 0) */
        //         {

        //             adjustmentValueLeft = MIN_MOVE + (Math.abs(offness) * ADJUST_PORP);
        //             adjustmentValueRight = MIN_MOVE - (Math.abs(offness) * ADJUST_PORP);

        //             Hardware.transmission.driveRaw(adjustmentValueLeft, adjustmentValueRight);
        //         }
        //         // else
        //         // {
        //         // // drives until reaches the distance to stop
        //         // Hardware.transmission.driveRaw(.2, .2);
        //         // }
        //     }
        // } else {
        //     System.out.println("done");
        //     // drive the remaining distance to the target
        //     if (Hardware.drive.driveStraightInches(10, .2, 0, true)) {
        //         // resets for the next run
        //         Hardware.drive.stop();
        //         secondTurn = false;
        //         setUp = false;
        //         return true;
        //     }
        // }
        return false;
    }

    final double MIN_MOVE = .15;

    final double ADJUST_PORP = .01;

}
