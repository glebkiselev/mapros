;; Auto-generated. Do not edit!


(when (boundp 'planner::AddRobotMessage)
  (if (not (find-package "PLANNER"))
    (make-package "PLANNER"))
  (shadow 'AddRobotMessage (find-package "PLANNER")))
(unless (find-package "PLANNER::ADDROBOTMESSAGE")
  (make-package "PLANNER::ADDROBOTMESSAGE"))
(unless (find-package "PLANNER::ADDROBOTMESSAGEREQUEST")
  (make-package "PLANNER::ADDROBOTMESSAGEREQUEST"))
(unless (find-package "PLANNER::ADDROBOTMESSAGERESPONSE")
  (make-package "PLANNER::ADDROBOTMESSAGERESPONSE"))

(in-package "ROS")





(defclass planner::AddRobotMessageRequest
  :super ros::object
  :slots (_request ))

(defmethod planner::AddRobotMessageRequest
  (:init
   (&key
    ((:request __request) "")
    )
   (send-super :init)
   (setq _request (string __request))
   self)
  (:request
   (&optional __request)
   (if __request (setq _request __request)) _request)
  (:serialization-length
   ()
   (+
    ;; string _request
    4 (length _request)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _request
       (write-long (length _request) s) (princ _request s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _request
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _request (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass planner::AddRobotMessageResponse
  :super ros::object
  :slots (_response ))

(defmethod planner::AddRobotMessageResponse
  (:init
   (&key
    ((:response __response) "")
    )
   (send-super :init)
   (setq _response (string __response))
   self)
  (:response
   (&optional __response)
   (if __response (setq _response __response)) _response)
  (:serialization-length
   ()
   (+
    ;; string _response
    4 (length _response)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _response
       (write-long (length _response) s) (princ _response s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _response
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _response (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass planner::AddRobotMessage
  :super ros::object
  :slots ())

(setf (get planner::AddRobotMessage :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get planner::AddRobotMessage :datatype-) "planner/AddRobotMessage")
(setf (get planner::AddRobotMessage :request) planner::AddRobotMessageRequest)
(setf (get planner::AddRobotMessage :response) planner::AddRobotMessageResponse)

(defmethod planner::AddRobotMessageRequest
  (:response () (instance planner::AddRobotMessageResponse :init)))

(setf (get planner::AddRobotMessageRequest :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get planner::AddRobotMessageRequest :datatype-) "planner/AddRobotMessageRequest")
(setf (get planner::AddRobotMessageRequest :definition-)
      "string request
---
string response

")

(setf (get planner::AddRobotMessageResponse :md5sum-) "33ea4e5aeb30f5913da681ca459d55f3")
(setf (get planner::AddRobotMessageResponse :datatype-) "planner/AddRobotMessageResponse")
(setf (get planner::AddRobotMessageResponse :definition-)
      "string request
---
string response

")



(provide :planner/AddRobotMessage "33ea4e5aeb30f5913da681ca459d55f3")


