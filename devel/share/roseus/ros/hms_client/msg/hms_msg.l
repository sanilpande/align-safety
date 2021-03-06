;; Auto-generated. Do not edit!


(when (boundp 'hms_client::hms_msg)
  (if (not (find-package "HMS_CLIENT"))
    (make-package "HMS_CLIENT"))
  (shadow 'hms_msg (find-package "HMS_CLIENT")))
(unless (find-package "HMS_CLIENT::HMS_MSG")
  (make-package "HMS_CLIENT::HMS_MSG"))

(in-package "ROS")
;;//! \htmlinclude hms_msg.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass hms_client::hms_msg
  :super ros::object
  :slots (_header _temp1 _temp2 ))

(defmethod hms_client::hms_msg
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:temp1 __temp1) "")
    ((:temp2 __temp2) 0.0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _temp1 (string __temp1))
   (setq _temp2 (float __temp2))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:temp1
   (&optional __temp1)
   (if __temp1 (setq _temp1 __temp1)) _temp1)
  (:temp2
   (&optional __temp2)
   (if __temp2 (setq _temp2 __temp2)) _temp2)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _temp1
    4 (length _temp1)
    ;; float32 _temp2
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _temp1
       (write-long (length _temp1) s) (princ _temp1 s)
     ;; float32 _temp2
       (sys::poke _temp2 (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _temp1
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _temp1 (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _temp2
     (setq _temp2 (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get hms_client::hms_msg :md5sum-) "64608a45b1add361d21d3cecf93db1f6")
(setf (get hms_client::hms_msg :datatype-) "hms_client/hms_msg")
(setf (get hms_client::hms_msg :definition-)
      "Header header
string temp1
float32 temp2
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :hms_client/hms_msg "64608a45b1add361d21d3cecf93db1f6")


