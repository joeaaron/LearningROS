;; Auto-generated. Do not edit!


(when (boundp 'test_msgs::Test)
  (if (not (find-package "TEST_MSGS"))
    (make-package "TEST_MSGS"))
  (shadow 'Test (find-package "TEST_MSGS")))
(unless (find-package "TEST_MSGS::TEST")
  (make-package "TEST_MSGS::TEST"))

(in-package "ROS")
;;//! \htmlinclude Test.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass test_msgs::Test
  :super ros::object
  :slots (_data _vel _pose _name ))

(defmethod test_msgs::Test
  (:init
   (&key
    ((:data __data) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:vel __vel) 0.0)
    ((:pose __pose) (instance geometry_msgs::Pose :init))
    ((:name __name) "")
    )
   (send-super :init)
   (setq _data __data)
   (setq _vel (float __vel))
   (setq _pose __pose)
   (setq _name (string __name))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:pose
   (&rest __pose)
   (if (keywordp (car __pose))
       (send* _pose __pose)
     (progn
       (if __pose (setq _pose (car __pose)))
       _pose)))
  (:name
   (&optional __name)
   (if __name (setq _name __name)) _name)
  (:serialization-length
   ()
   (+
    ;; float32[] _data
    (* 4    (length _data)) 4
    ;; float32 _vel
    4
    ;; geometry_msgs/Pose _pose
    (send _pose :serialization-length)
    ;; string _name
    4 (length _name)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[] _data
     (write-long (length _data) s)
     (dotimes (i (length _data))
       (sys::poke (elt _data i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; float32 _vel
       (sys::poke _vel (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; geometry_msgs/Pose _pose
       (send _pose :serialize s)
     ;; string _name
       (write-long (length _name) s) (princ _name s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[] _data
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _data (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _data i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; float32 _vel
     (setq _vel (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; geometry_msgs/Pose _pose
     (send _pose :deserialize buf ptr-) (incf ptr- (send _pose :serialization-length))
   ;; string _name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get test_msgs::Test :md5sum-) "57d24e0384e259f3b08f5a699fa7b1a0")
(setf (get test_msgs::Test :datatype-) "test_msgs/Test")
(setf (get test_msgs::Test :definition-)
      "float32[] data
float32 vel
geometry_msgs/Pose pose
string name

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :test_msgs/Test "57d24e0384e259f3b08f5a699fa7b1a0")


