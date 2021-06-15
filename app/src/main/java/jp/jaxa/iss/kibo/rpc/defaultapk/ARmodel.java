package jp.jaxa.iss.kibo.rpc.defaultapk;



import java.util.ArrayList;

import org.opencv.aruco.Aruco;
import org.opencv.aruco.Dictionary;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class ARmodel {

    private static int dictID = Aruco.DICT_5X5_250;
    private static Mat ids;
    private static ArrayList<Mat> corners;
    private static Dictionary dict;
    private static Scalar borderColor;


    private static float markerSize;

    private static Mat rvecs;
    private static Mat tvecs;
    private static Mat mean_rvecs;
    private static Mat rotationMatrix;

    private static Point3 target_pos=new Point3();

    private boolean isID(int index,int id) {
        return ids.get(index,0)[0] == id;
    }

    public float getPosX() {
        return (float)target_pos.x;
    }

    public float getPosY() {
        return (float)target_pos.y *(-1.0f);
    }

    public float getPosZ() {
        return (float)target_pos.z *(-1.0f);
    }

    public void estimateTest(Mat img, Mat camMatrix, Mat dstMatrix) {
        long total_time = 0;
        int loop_cnt = 50;
        for (int i = 0; i < loop_cnt; i++) {
            long start_time = System.currentTimeMillis();
            estimateV2(img, camMatrix, dstMatrix);
            long end_time = System.currentTimeMillis();
            long diff_time = end_time - start_time;
            total_time += diff_time;
//            System.out.printf("%d: AR time : %d ms\n", i, (diff_time) * 200);
        }


//        System.out.printf("AR time : %d ms\n", (total_time) * 200 / loop_cnt);
    }

    public void estimateV2(Mat img,Mat camMatrix,Mat dstMatrix) {


        corners = new ArrayList<>();
        ids = new Mat();
        dict = Aruco.getPredefinedDictionary(dictID);
        Aruco.detectMarkers(img ,dict ,corners,ids);

        borderColor = new Scalar (255.0, 0.0, 0.0);

        if(ids.size().height>0) {

//			Aruco.drawDetectedMarkers(img , corners, ids, borderColor);		// draw square @ Markers

            markerSize=0.05f;
            rotationMatrix = new Mat ();
            rvecs = new Mat();
            tvecs = new Mat();
            Aruco.estimatePoseSingleMarkers(corners, markerSize, camMatrix, dstMatrix, rvecs, tvecs);


            int id_cnt =0;


            for(int i=0;i<rvecs.size().height;i++) {

                id_cnt++;
                Mat rvec=new Mat(1,3,CvType.CV_64FC1);
                Mat tvec=new Mat(1,3,CvType.CV_64FC1);
                Mat rot=new Mat();
                rvec.put(0, 0, rvecs.get(i,0));
                tvec.put(0, 0, tvecs.get(i,0));

//				Calib3d.drawFrameAxes(img, camMatrix, dstMatrix, rvec, tvec, (float)0.15); 	// draw 3axis @ Markers


                float pos_x=(float)tvec.get(0, 0)[0];
                float pos_y=(float)tvec.get(0, 1)[0];
                float pos_z=(float)tvec.get(0, 2)[0];


                float x_offset = 0.1125f;
                float y_offset = 0.0425f;

                if(isID(i,1)) {
                    x_offset *= -1;
                    y_offset *= -1;
                }
                else if(isID(i,2)) {
                    y_offset *= -1;
                }
                else if(isID(i,4)) {
                    x_offset *= -1;
                }

                target_pos.x=target_pos.x+pos_x + x_offset;
                target_pos.y=target_pos.y+pos_y + y_offset;
                target_pos.z=target_pos.z+pos_z;


//                String AR_position = String.format("id %s\t(%.2f,%.2f,%.2f)m",(int)ids.get(i,0)[0],pos_x,pos_y,pos_z);


//				convert rvec to rotation matrix
                Calib3d.Rodrigues(rvec, rot);

                quaternion orientation = new quaternion();

                orientation.fromRotationMatrix(rot);
                orientation.toEuler();
//				System.out.println(rot.dump());
//				System.out.printf("orien : %.3f %.3f %.3f %.3f\n",orientation.x,orientation.y,orientation.z,orientation.w);
//				System.out.printf("orien : %.3f %.3f %.3f\n",orientation.roll,orientation.pitch,orientation.yaw);

                //System.out.println((int)ids.get(i,0)[0]);

//				System.out.println(AR_position);
            }

//			System.out.printf("ID counting: %d\n",id_cnt);

            target_pos.x=target_pos.x/id_cnt;
            target_pos.y=target_pos.y/id_cnt;
            target_pos.z=target_pos.z/id_cnt;


//			String target_position = String.format("(%.2f,%.2f,%.2f)m",target_pos.x,target_pos.y,target_pos.z);

//			System.out.printf("center\t%s\n",target_position);
        }
    }
}