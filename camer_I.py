import cv2 as cv
from cv2 import aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
from operations.py import build_tf,get_pose,multiply_mat,quaternion_rotation_matrix


flag2=0

calib_data_path = "/home/rbccps-kuka/Downloads/kuka_ros/Basic-Augmented-reality-course-opencv-master/calib_data/MultiMatrixlenovo.npz"

calib_data = np.load(calib_data_path)
print(calib_data.files)

cam_mat = calib_data["camMatrix"]
dist_coef = calib_data["distCoef"]
r_vectors = calib_data["rVector"]
t_vectors = calib_data["tVector"]
MARKER_SIZE = 3.5  # centimeters

marker_dict = aruco.Dictionary_get(aruco.DICT_5X5_50)

param_markers = aruco.DetectorParameters_create()


def get_avbl_beakers():
    cap = cv.VideoCapture(2)
    flag2=0
    while True:
        ret, frame = cap.read()
        flag2+=1
        if not ret:
            break
        print(1234)
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        if marker_corners:
            print(123)
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                flat_list = [item for sublist in marker_IDs for item in sublist]
                print(flat_list)
                
                if '2' in flat_list:
                    flat_list.remove('2')
                if '34' in flat_list:
                    flat_list.remove('34')                # def on_publish(client,userdata,result):             #create function for callback

                listToStr = ' '.join([str(elem) for elem in flat_list])
                print(flat_list)
                print(listToStr)
                  
                if flag2==10:                         #establish connection


                    cap.release()
                    cv.destroyAllWindows()
                    return listToStr
            
                    

                    
        cv.imshow("frame", frame)
        key = cv.waitKey(1)
        if key == ord("q"):
            break
    cap.release()
    cv.destroyAllWindows()



def get_tf(id,tutorial):
    cap = cv.VideoCapture(2)
    tf_ttc = np.array([[0, 0, 0,0],
                                [0, 0, 0,0],
                                [0, 0, 0,0],
                                [0,0,0,1]],
                                dtype=float)

    flag=0
    while True:
        print("abc")
        ret, frame = cap.read()
        # frame = cv.rotate(frame, cv.ROTATE_90_COUNTERCLOCKWISE)
        # frame = cv.flip(frame,0)
        if not ret:
            break
        gray_frame = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        flat_list = [item for sublist in marker_IDs for item in sublist]
        
        print(type(marker_IDs),'abc')
        if marker_corners and id in flat_list:
            index=flat_list.index(id)
            print(id,flat_list,index)

            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(            # ser=serial.Serial("/dev/ttyACM0",9600,timeout=1)
            # ser.write(b'b')
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            tf_ttc[0][3]=tVec[index][0][0]/100
            tf_ttc[1][3]=tVec[index][0][1]/100
            tf_ttc[2][3]=tVec[index][0][2]/100
            print(tf_ttc[0][3])
            print(rVec)
            tf_ttc[:3,:3], _ = cv.Rodrigues(rVec[len(flat_list)-index-1])
            print("----------------------")
            print("TF-TTC")
            # tf_ett=build_tf(rotation_matrix,np.transpose(tVec))
            print(np.array(tf_ttc))
            print("----------------------")
            handtobase = tutorial.handtobase()

            htb_qt=[handtobase.orientation.x,handtobase.orientation.y,handtobase.orientation.z,handtobase.orientation.w]
            htb_tr=[[handtobase.position.x,handtobase.position.y,handtobase.position.z]]
            tf_etb=get_pose(htb_tr,htb_qt)
            print("TF-ETB")
            print(np.array(tf_etb))
            print("-----------------------")
            tf_btc = np.array([[0.99886654 , 0.0453864 ,-0.01434255 ,0.70454882],
                            [0.00907929 , 0.11411491 , 0.99342607,-0.57296331],
                            [0.04672473 ,-0.99243028 , 0.11357348,0.44726041],
                            [0,0,0,1]],
                            dtype=float)
            print("TF-bTC")
            print(np.array(tf_btc))

            final_tf=multiply_mat(tf_btc,np.array(tf_ttc))
            final_rot=final_tf[:3,:3]
            r=R.from_matrix(final_rot)
            final_quat=r.as_quat()
            print("Final_TF")
            print(final_tf)
            flag+=1
            cap.release()
            cv.destroyAllWindows()





            # print(euler_from_quaternion(quaternion[0],quaternion[1],quaternion[2],quaternion[3]))
            total_markers = range(0, marker_IDs.size)
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                cv.polylines(
                    frame, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Since there was mistake in calculating the distance approach point-outed in the Video Tutorial's comment
                # so I have rectified that mistake, I have test that out it increase the accuracy overall.
                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )
                # Draw the pose of the marker
                point = cv.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                cv.putText(frame,f"id: {ids[0]} Dist: {round(distance, 2)}",top_right,cv.FONT_HERSHEY_PLAIN,1.3,(0, 0, 255),2,cv.LINE_AA,
                )
                cv.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv.FONT_HERSHEY_PLAIN,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv.LINE_AA,
                )






                # print(ids, "  ", corners)
        cv.imshow("frame", frame)
        while(flag==1):
            return final_tf,final_quat
        key = cv.waitKey(1)
        if key == ord("q"):
            break
        cap.release()
        cv.destroyAllWindows()
    

