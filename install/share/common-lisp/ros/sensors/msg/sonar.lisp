; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude sonar.msg.html

(cl:defclass <sonar> (roslisp-msg-protocol:ros-message)
  ((id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0)
   (distance
    :reader distance
    :initarg :distance
    :type (cl:vector cl:integer)
   :initform (cl:make-array 10 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass sonar (<sonar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <sonar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'sonar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<sonar> is deprecated: use sensors-msg:sonar instead.")))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:id-val is deprecated.  Use sensors-msg:id instead.")
  (id m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <sonar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:distance-val is deprecated.  Use sensors-msg:distance instead.")
  (distance m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <sonar>) ostream)
  "Serializes a message object of type '<sonar>"
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
   (cl:slot-value msg 'distance))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <sonar>) istream)
  "Deserializes a message object of type '<sonar>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:setf (cl:slot-value msg 'distance) (cl:make-array 10))
  (cl:let ((vals (cl:slot-value msg 'distance)))
    (cl:dotimes (i 10)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<sonar>)))
  "Returns string type for a message object of type '<sonar>"
  "sensors/sonar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'sonar)))
  "Returns string type for a message object of type 'sonar"
  "sensors/sonar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<sonar>)))
  "Returns md5sum for a message object of type '<sonar>"
  "b89e6bd15cc9882501626a42b8d207ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'sonar)))
  "Returns md5sum for a message object of type 'sonar"
  "b89e6bd15cc9882501626a42b8d207ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<sonar>)))
  "Returns full string definition for message of type '<sonar>"
  (cl:format cl:nil "int32 id~%int32[10] distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'sonar)))
  "Returns full string definition for message of type 'sonar"
  (cl:format cl:nil "int32 id~%int32[10] distance~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <sonar>))
  (cl:+ 0
     4
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'distance) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <sonar>))
  "Converts a ROS message object to a list"
  (cl:list 'sonar
    (cl:cons ':id (id msg))
    (cl:cons ':distance (distance msg))
))
