; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude motorArray.msg.html

(cl:defclass <motorArray> (roslisp-msg-protocol:ros-message)
  ((motors
    :reader motors
    :initarg :motors
    :type (cl:vector sensors-msg:motor)
   :initform (cl:make-array 0 :element-type 'sensors-msg:motor :initial-element (cl:make-instance 'sensors-msg:motor))))
)

(cl:defclass motorArray (<motorArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <motorArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'motorArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<motorArray> is deprecated: use sensors-msg:motorArray instead.")))

(cl:ensure-generic-function 'motors-val :lambda-list '(m))
(cl:defmethod motors-val ((m <motorArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:motors-val is deprecated.  Use sensors-msg:motors instead.")
  (motors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <motorArray>) ostream)
  "Serializes a message object of type '<motorArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <motorArray>) istream)
  "Deserializes a message object of type '<motorArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'sensors-msg:motor))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<motorArray>)))
  "Returns string type for a message object of type '<motorArray>"
  "sensors/motorArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'motorArray)))
  "Returns string type for a message object of type 'motorArray"
  "sensors/motorArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<motorArray>)))
  "Returns md5sum for a message object of type '<motorArray>"
  "68cccbaf2eb185196993c23e77c41a28")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'motorArray)))
  "Returns md5sum for a message object of type 'motorArray"
  "68cccbaf2eb185196993c23e77c41a28")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<motorArray>)))
  "Returns full string definition for message of type '<motorArray>"
  (cl:format cl:nil "motor[] motors~%~%================================================================================~%MSG: sensors/motor~%int32 id~%int32 rpm~%int32 temp~%int32 volt~%int32 curr~%int32 dir~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'motorArray)))
  "Returns full string definition for message of type 'motorArray"
  (cl:format cl:nil "motor[] motors~%~%================================================================================~%MSG: sensors/motor~%int32 id~%int32 rpm~%int32 temp~%int32 volt~%int32 curr~%int32 dir~%int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <motorArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <motorArray>))
  "Converts a ROS message object to a list"
  (cl:list 'motorArray
    (cl:cons ':motors (motors msg))
))
