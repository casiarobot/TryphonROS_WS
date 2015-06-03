; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude compass.msg.html

(cl:defclass <compass> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (rz
    :reader rz
    :initarg :rz
    :type (cl:vector cl:integer)
   :initform (cl:make-array 10 :element-type 'cl:integer :initial-element 0))
   (head_init
    :reader head_init
    :initarg :head_init
    :type cl:integer
    :initform 0))
)

(cl:defclass compass (<compass>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <compass>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'compass)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<compass> is deprecated: use sensors-msg:compass instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:id-val is deprecated.  Use sensors-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'rz-val :lambda-list '(m))
(cl:defmethod rz-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:rz-val is deprecated.  Use sensors-msg:rz instead.")
  (rz m))

(cl:ensure-generic-function 'head_init-val :lambda-list '(m))
(cl:defmethod head_init-val ((m <compass>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:head_init-val is deprecated.  Use sensors-msg:head_init instead.")
  (head_init m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <compass>) ostream)
  "Serializes a message object of type '<compass>"
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'rz))
  (cl:let* ((signed (cl:slot-value msg 'head_init)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <compass>) istream)
  "Deserializes a message object of type '<compass>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'rz) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'rz)))
    (cl:dotimes (i 10)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'head_init) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<compass>)))
  "Returns string type for a message object of type '<compass>"
  "sensors/compass")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'compass)))
  "Returns string type for a message object of type 'compass"
  "sensors/compass")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<compass>)))
  "Returns md5sum for a message object of type '<compass>"
  "4b9d9eecbee8c0fcf7f1f2c957b3f53e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'compass)))
  "Returns md5sum for a message object of type 'compass"
  "4b9d9eecbee8c0fcf7f1f2c957b3f53e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<compass>)))
  "Returns full string definition for message of type '<compass>"
  (cl:format cl:nil "int32 id~%int32[10] rz~%int32 head_init~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'compass)))
  "Returns full string definition for message of type 'compass"
  (cl:format cl:nil "int32 id~%int32[10] rz~%int32 head_init~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <compass>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'rz) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <compass>))
  "Converts a ROS message object to a list"
  (cl:list 'compass
    (cl:cons ':id (id msg))
    (cl:cons ':rz (rz msg))
    (cl:cons ':head_init (head_init msg))
))
