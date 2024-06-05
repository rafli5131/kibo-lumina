package jp.jaxa.iss.kibo.rpc.sampleapk;

import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
import gov.nasa.arc.astrobee.Result;
import gov.nasa.arc.astrobee.types.Point;
import gov.nasa.arc.astrobee.types.Quaternion;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.aruco.Aruco;
import org.opencv.aruco.DetectorParameters;
import org.opencv.aruco.Dictionary;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import android.util.Log;
import android.util.SparseArray;
import android.util.SparseIntArray;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

@SuppressWarnings("SpellCheckingInspection")
public class YourService extends KiboRpcService {

    private Mat camMat, distCoeff;
    private List<Integer> activeTargets;
    private List<Integer> prevTargets = new ArrayList<>();
    private SparseArray<Coordinate> targetList;
    private SparseArray<List<Integer>> parentIDInfo;
    private SparseIntArray pointMap, quickMoves;
    private String mQrContent = "No QR Content could be found.";
    private int currParent = 0, phase = 1;

    private static final String
            TAG = "CARTOGRAPHER",
            SIM = "Simulator",
            IRL = "Orbit";

    @Override
    @SuppressWarnings("all")
    protected void runPlan1() {
        // record startTime
        long startTime = System.currentTimeMillis();

        // initialize data and start mission
        api.startMission();
        initCam(SIM);
        initMaps();

        // handle movement to targets and points per phase optimization
        while (api.getTimeRemaining().get(1) > 80000 && phase <= 3) {
            Log.i(TAG, "Entered phase #" + phase);
            int targetToHit = 0;

            if (api.getTimeRemaining().get(1) <= 80000 && phase == 3) {
                Log.i(TAG, "Breaking loop to prioritize time");
                break;
            }

            activeTargets = api.getActiveTargets();
            Log.i(TAG, "Active Targets: " + activeTargets.toString());

            targetToHit = ListUtils.findMaxFromMap(activeTargets, pointMap);

            if (activeTargets.size() > 1)
                targetToHit = STRATEGIZE(targetToHit);

            if (api.getTimeRemaining().get(1) < 80000) break;

            Log.i(TAG, "Going for target #" + targetToHit);
            moveTo(targetList.get(targetToHit), true, false);

            Log.i(TAG, "Time Remaining After Target #" + targetToHit + ": " +
                    api.getTimeRemaining().get(1) + "ms");

            Log.i(TAG, "Previous targets: " + prevTargets.toString());
            phase++;
        }
        Log.i(TAG, "Broke out of loop with " + api.getTimeRemaining().get(1) + "ms remaining");

        // handle QR code
        if (prevTargets.get(prevTargets.size() - 1) != 4 && api.getTimeRemaining().get(1) > 80000) {
            Log.i(TAG, "Heading to QR Code");
            moveTo(targetList.get(7), false, true); // Time Cost: 1m10s
        } else {
            scanQR(false);
            Log.i(TAG, "Skipped QR Code to prioritize time.");
        }
        Log.i(TAG, "QR Content: " + mQrContent);

        // notify that astrobee is heading to the goal
        api.notifyGoingToGoal();
        moveTo(targetList.get(8), false, false); // Time Cost: 45s

        // send mission completion and record deltaTime
        long deltaTime = System.currentTimeMillis() - startTime;
        Log.i(TAG, "runPlan1() executed in: " + deltaTime / 1000 + "s.");
        api.reportMissionCompletion(mQrContent);
    }

    @Override
    protected void runPlan2() { /* write your plan 2 here */ }

    @Override
    protected void runPlan3() { /* write your plan 3 here */ }

    private int moveTo(Coordinate coordinate, boolean scanTag, boolean QR) {
        int target = targetList.indexOfValue(coordinate);

        if (coordinate.hasParent()) {
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent);
        }

        moveTo(coordinate.getPoint(), coordinate.getQuaternion());
        if (scanTag) targetLaser(target);
        else if (QR) mQrContent = scanQR(false);

        if (coordinate.hasParent() && (target != 4) && (target != 8)) {
            Coordinate parent = coordinate.getParent();
            moveTo(parent, false, false);
            currParent = parent.parentID;
            Log.i(TAG, "Current Parent ID: " + currParent);
        }

        return target;
    }

    private void moveTo(Point point, Quaternion quaternion) {
        final int LOOP_MAX = 10;

        Log.i(TAG, "Moving to: " + point.getX() + ", " + point.getY() + ", " + point.getZ());
        long start = System.currentTimeMillis();

        Result result = api.moveTo(point, quaternion, true);

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "[0] moveTo finished in : " + elapsedTime / 1000 + " seconds");
        Log.i(TAG, "[0] hasSucceeded : " + result.hasSucceeded());

        int loopCounter = 1;
        while (!result.hasSucceeded() && loopCounter <= LOOP_MAX) {

            Log.i(TAG, "[" + loopCounter + "] " + "Calling moveTo function");
            start = System.currentTimeMillis();

            result = api.moveTo(point, quaternion, true);

            end = System.currentTimeMillis();
            elapsedTime = end - start;
            Log.i(TAG, "[" + loopCounter + "] " + "moveTo finished in : " + elapsedTime / 1000 +
                    " seconds");
            Log.i(TAG, "[" + loopCounter + "] " + "hasSucceeded : " + result.hasSucceeded());

            loopCounter++;
        }
    }

    private void moveTo(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w) {
        moveTo(new Point(pt_x, pt_y, pt_z), new Quaternion(q_x, q_y, q_z, q_w));
    }

    private void initCam(String mode) {
        camMat = new Mat(3, 3, CvType.CV_32F);
        distCoeff = new Mat(1, 5, CvType.CV_32F);

        if (mode.equals(SIM)) {
            float[] camArr = {
                    661.783002f, 0.000000f, 595.212041f,
                    0.000000f, 671.508662f, 489.094196f,
                    0.000000f, 0.000000f, 1.000000f
            };
            float[] distortionCoefficients = {
                    -0.215168f, 0.044354f, 0.003615f, 0.005093f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0, 0, distortionCoefficients);
        } else if (mode.equals(IRL)) {
            float[] camArr = {
                    753.51021f, 0.0f, 631.11512f,
                    0.0f, 751.3611f, 508.69621f,
                    0.0f, 0.0f, 1.0f
            };
            float[] distortionCoefficients = {
                    -0.411405f, 0.177240f, -0.017145f, 0.006421f, 0.000000f
            };

            camMat.put(0, 0, camArr);
            distCoeff.put(0, 0, distortionCoefficients);
        }
        Log.i(TAG, "Initialized Camera Matrices in Mode: " + mode);
    }

    private int getTagInfo(int tagNum) {
        Log.i(TAG, "Calling getTagInfo() function");
        long start = System.currentTimeMillis();

        Mat undistorted = new Mat();
        Mat ids = new Mat();

        api.flashlightControlFront(0.05f); // enable flashlight for tag read clarity
        Mat distorted = api.getMatNavCam();
        api.flashlightControlFront(0.00f);

        Imgproc.undistort(distorted, undistorted, camMat, distCoeff);
        Log.i(TAG, "Undistorted Image Successfully");

        Dictionary dict = Aruco.getPredefinedDictionary(Aruco.DICT_4X4_250);
        DetectorParameters param = DetectorParameters.create();

        ArrayList<Mat> corners = new ArrayList<>();
        Aruco.detectMarkers(undistorted, dict, corners, ids, param);

        Log.i(TAG, "Detected Markers: " + ids.dump());
        undistorted.release();

        int id = -1;
        if (corners.size() != 0) {
            for (int i = 0; i < ids.rows(); i++) {
                id = (int) ids.get(i, 0)[0];

                if (id == tagNum) {
                    Log.i(TAG, "Found Target AR Tag: " + id);
                    break;
                }
            }
        }

        long end = System.currentTimeMillis();
        long elapsedTime = end - start;
        Log.i(TAG, "getTagInfo() executed in: " + elapsedTime / 1000 + "s.");
        return id;
    }

    private String scanQR(boolean gotoQR) {
        if (gotoQR) moveTo(targetList.get(7), false, false);

        Mat gray = new Mat();
        Mat frame = api.getMatNavCam();
        Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGB2GRAY);

        QRCodeDetector qrCodeDetector = new QRCodeDetector();
        String qrContent = qrCodeDetector.detectAndDecode(gray);
        gray.release();

        if (!qrContent.isEmpty()) mQrContent = qrContent;

        return qrContent;
    }

    private void targetLaser(int target) {
        api.laserControl(true);
        api.takeTargetSnapshot(target);
        prevTargets.add(target);
        api.laserControl(false);
    }

    private void initMaps() {
        quickMoves = new SparseIntArray() {
            {
                put(0, 1); // default spawn parent
                put(4, 5); // QR parent
                put(8, 0); // endpoint parent
            }
        };

        parentIDInfo = new SparseArray<List<Integer>>() {
            {
                put(1, new ArrayList<Integer>() {{
                    add(3); add(6); add(8);
                }});
                put(5, new ArrayList<Integer>() {{
                    add(4); add(7); add(8);
                }});
            }
        };

        targetList = new SparseArray<Coordinate>() {
            {
                put(0, new Coordinate(10.71, -7.7, 4.48, 0f, 0f, -0.707f, 0.707f)); // start point
                put(1, new Coordinate(10.71, -9.8, 4.48, 0f, 0f, -0.707f, 0.707f, 0));
                put(2, new Coordinate(10.71, -11.3, 4.48, 0f, 0f, -0.707f, 0.707f, 0));
                put(3, new Coordinate(10.71, -12.8, 4.48, 0f, 0f, -0.707f, 0.707f, 0));
                put(4, new Coordinate(11.21, -10.9, 4.48, 0f, 0f, -0.707f, 0.707f)); // QR Code Point
                put(5, new Coordinate(10.41, -8.9, 4.48, 0f, 0f, -0.707f, 0.707f, 0));
                put(6, new Coordinate(10.41, -10.4, 4.48, 0f, 0f, -0.707f, 0.707f, 0));
                put(7, new Coordinate(11.21, -10.4, 4.48, 0f, 0f, -0.707f, 0.707f)); // QR Code
                put(8, new Coordinate(11.21, -8.4, 4.48, 0f, 0f, -0.707f, 0.707f)); // end point
            }
        };

        pointMap = new SparseIntArray() {
            {
                put(1, 10);
                put(2, 10);
                put(3, 10);
                put(5, 8);
                put(6, 8);
                put(4, 15); // QR Code point
                put(8, 5);
            }
        };
    }

    private int STRATEGIZE(int targetToHit) {
        int replacementTarget = 0;
        Random rand = new Random();

        List<Integer> adj = parentIDInfo.get(currParent);
        if (adj != null) {
            for (int x = 0; x < adj.size(); x++) {
                int poss = adj.get(x);

                if (targetToHit == 4 && poss == 4) return targetToHit;

                if (targetToHit != 4 && poss != 4) replacementTarget = adj.get(rand.nextInt(adj.size()));
                if (replacementTarget != 4) break;
            }
        }
        Log.i(TAG, "Strategized Target: " + replacementTarget);
        return replacementTarget;
    }

    static class Coordinate {
        private final Point mPoint;
        private final Quaternion mQuaternion;
        private final Coordinate mParent;
        private final int parentID;

        public Coordinate(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w) {
            this.mPoint = new Point(pt_x, pt_y, pt_z);
            this.mQuaternion = new Quaternion(q_x, q_y, q_z, q_w);
            this.mParent = null;
            this.parentID = 0;
        }

        public Coordinate(double pt_x, double pt_y, double pt_z, float q_x, float q_y, float q_z, float q_w, int parentID) {
            this.mPoint = new Point(pt_x, pt_y, pt_z);
            this.mQuaternion = new Quaternion(q_x, q_y, q_z, q_w);
            this.parentID = parentID;
            this.mParent = YourService.targetList.get(YourService.quickMoves.get(parentID));
        }

        public Point getPoint() { return mPoint; }
        public Quaternion getQuaternion() { return mQuaternion; }
        public boolean hasParent() { return mParent != null; }
        public Coordinate getParent() { return mParent; }
    }

    private static class ListUtils {
        public static int findMaxFromMap(List<Integer> list, SparseIntArray map) {
            int currMax = 0;
            int currMaxIndex = 0;

            for (int i = 0; i < list.size(); i++) {
                int element = list.get(i);
                int points = map.get(element);

                if (points > currMax) {
                    currMax = points;
                    currMaxIndex = element;
                }
            }

            return currMaxIndex;
        }
    }
}
