; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude sonarArray.msg.html

(cl:defclass <sonarArray> (roslisp-msg-protocol:ros-message)
  ((sonars
    :reader sonars
    :initarg :sonars
    :type (cl:vector sensors-msg:sonar)
   :initform (cl:make-array 0 :element-type 'sensors-msg:sonar :initial-element (cl:make-instance 'sensors-msg:sonar))))
)

(cl:defclass sonarArray (<sonarArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sonarArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sonarArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<sonarArray> is deprecated: use sensors-msg:sonarArray instead.")))

(cl:ensure-generic-function 'sonars-val :lambda-list '(m))
(cl:defmethod sonars-val ((m <sonarArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:sonars-val is deprecated.  Use sensors-msg:sonars instead.")
  (sonars m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sonarArray>) ostream)
  "Serializes a message object of type '<sonarArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'sonars))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'sonars))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sonarArray>) istream)
  "Deserializes a message object of type '<sonarArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'sonars) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'sonars)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensors-msg:sonar))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sonarArray>)))
  "Returns string type for a message object of type '<sonarArray>"
  "sensors/sonarArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sonarArray)))
  "Returns string type for a message object of type 'sonarArray"
  "sensors/sonarArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sonarArray>)))
  "Returns md5sum for a message object of type '<sonarArray>"
  "afeb67b4dccb598c2a418c6a2f41b00b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sonarArray)))
  "Returns md5sum for a message object of type 'sonarArray"
  "afeb67b4dccb598c2a418c6a2f41b00b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sonarArray>)))
  "Returns full string definition for message of type '<sonarArray>"
  (cl:format cl:nil "sonar[] sonars~%~%================================================================================~%MSG: sensors/sonar~%int32 id~%int32[10] distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sonarArray)))
  "Returns full string definition for message of type 'sonarArray"
  (cl:format cl:nil "sonar[] sonars~%~%================================================================================~%MSG: sensors/sonar~%int32 id~%int32[10] distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sonarArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'sonars) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sonarArray>))
  "Converts a ROS message object to a list"
  (cl:list 'sonarArray
    (cl:cons ':sonars (sonars msg))
))
