; Auto-generated. Do not edit!


(cl:in-package planner-srv)


;//! \htmlinclude AddRobotMessage-request.msg.html

(cl:defclass <AddRobotMessage-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:string
    :initform ""))
)

(cl:defclass AddRobotMessage-request (<AddRobotMessage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddRobotMessage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddRobotMessage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-srv:<AddRobotMessage-request> is deprecated: use planner-srv:AddRobotMessage-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <AddRobotMessage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-srv:request-val is deprecated.  Use planner-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddRobotMessage-request>) ostream)
  "Serializes a message object of type '<AddRobotMessage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'request))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'request))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddRobotMessage-request>) istream)
  "Deserializes a message object of type '<AddRobotMessage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'request) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddRobotMessage-request>)))
  "Returns string type for a service object of type '<AddRobotMessage-request>"
  "planner/AddRobotMessageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddRobotMessage-request)))
  "Returns string type for a service object of type 'AddRobotMessage-request"
  "planner/AddRobotMessageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddRobotMessage-request>)))
  "Returns md5sum for a message object of type '<AddRobotMessage-request>"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddRobotMessage-request)))
  "Returns md5sum for a message object of type 'AddRobotMessage-request"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddRobotMessage-request>)))
  "Returns full string definition for message of type '<AddRobotMessage-request>"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddRobotMessage-request)))
  "Returns full string definition for message of type 'AddRobotMessage-request"
  (cl:format cl:nil "string request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddRobotMessage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'request))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddRobotMessage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'AddRobotMessage-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude AddRobotMessage-response.msg.html

(cl:defclass <AddRobotMessage-response> (roslisp-msg-protocol:ros-message)
  ((response
    :reader response
    :initarg :response
    :type cl:string
    :initform ""))
)

(cl:defclass AddRobotMessage-response (<AddRobotMessage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AddRobotMessage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AddRobotMessage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name planner-srv:<AddRobotMessage-response> is deprecated: use planner-srv:AddRobotMessage-response instead.")))

(cl:ensure-generic-function 'response-val :lambda-list '(m))
(cl:defmethod response-val ((m <AddRobotMessage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader planner-srv:response-val is deprecated.  Use planner-srv:response instead.")
  (response m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AddRobotMessage-response>) ostream)
  "Serializes a message object of type '<AddRobotMessage-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'response))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'response))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AddRobotMessage-response>) istream)
  "Deserializes a message object of type '<AddRobotMessage-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'response) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'response) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AddRobotMessage-response>)))
  "Returns string type for a service object of type '<AddRobotMessage-response>"
  "planner/AddRobotMessageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddRobotMessage-response)))
  "Returns string type for a service object of type 'AddRobotMessage-response"
  "planner/AddRobotMessageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AddRobotMessage-response>)))
  "Returns md5sum for a message object of type '<AddRobotMessage-response>"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AddRobotMessage-response)))
  "Returns md5sum for a message object of type 'AddRobotMessage-response"
  "33ea4e5aeb30f5913da681ca459d55f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AddRobotMessage-response>)))
  "Returns full string definition for message of type '<AddRobotMessage-response>"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AddRobotMessage-response)))
  "Returns full string definition for message of type 'AddRobotMessage-response"
  (cl:format cl:nil "string response~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AddRobotMessage-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'response))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AddRobotMessage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'AddRobotMessage-response
    (cl:cons ':response (response msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'AddRobotMessage)))
  'AddRobotMessage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'AddRobotMessage)))
  'AddRobotMessage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AddRobotMessage)))
  "Returns string type for a service object of type '<AddRobotMessage>"
  "planner/AddRobotMessage")