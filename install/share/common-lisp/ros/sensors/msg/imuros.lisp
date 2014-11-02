; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude imuros.msg.html

(cl:defclass <imuros> (roslisp-msg-protocol:ros-message)
  ((accel
    :reader accel
    :initarg :accel
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (gyro
    :reader gyro
    :initarg :gyro
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (magn
    :reader magn
    :initarg :magn
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass imuros (<imuros>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imuros>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imuros)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<imuros> is deprecated: use sensors-msg:imuros instead.")))

(cl:ensure-generic-function 'accel-val :lambda-list '(m))
(cl:defmethod accel-val ((m <imuros>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:accel-val is deprecated.  Use sensors-msg:accel instead.")
  (accel m))

(cl:ensure-generic-function 'gyro-val :lambda-list '(m))
(cl:defmethod gyro-val ((m <imuros>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:gyro-val is deprecated.  Use sensors-msg:gyro instead.")
  (gyro m))

(cl:ensure-generic-function 'magn-val :lambda-list '(m))
(cl:defmethod magn-val ((m <imuros>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:magn-val is deprecated.  Use sensors-msg:magn instead.")
  (magn m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imuros>) ostream)
  "Serializes a message object of type '<imuros>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'accel))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'gyro))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'magn))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imuros>) istream)
  "Deserializes a message object of type '<imuros>"
  (cl:setf (cl:slot-value msg 'accel) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'accel)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'gyro) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'gyro)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'magn) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'magn)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imuros>)))
  "Returns string type for a message object of type '<imuros>"
  "sensors/imuros")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imuros)))
  "Returns string type for a message object of type 'imuros"
  "sensors/imuros")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imuros>)))
  "Returns md5sum for a message object of type '<imuros>"
  "b01df9aeabf2ec2a45aad9d35856770e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imuros)))
  "Returns md5sum for a message object of type 'imuros"
  "b01df9aeabf2ec2a45aad9d35856770e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imuros>)))
  "Returns full string definition for message of type '<imuros>"
  (cl:format cl:nil "float64[3] accel~%float64[3] gyro~%float64[3] magn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imuros)))
  "Returns full string definition for message of type 'imuros"
  (cl:format cl:nil "float64[3] accel~%float64[3] gyro~%float64[3] magn~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imuros>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'accel) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gyro) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'magn) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imuros>))
  "Converts a ROS message object to a list"
  (cl:list 'imuros
    (cl:cons ':accel (accel msg))
    (cl:cons ':gyro (gyro msg))
    (cl:cons ':magn (magn msg))
))
