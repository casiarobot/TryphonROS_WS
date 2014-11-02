; Auto-generated. Do not edit!


(cl:in-package sensors-msg)


;//! \htmlinclude forces.msg.html

(cl:defclass <forces> (roslisp-msg-protocol:ros-message)
  ((target
    :reader target
    :initarg :target
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0))
   (current
    :reader current
    :initarg :current
    :type (cl:vector cl:float)
   :initform (cl:make-array 6 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass forces (<forces>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <forces>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'forces)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name sensors-msg:<forces> is deprecated: use sensors-msg:forces instead.")))

(cl:ensure-generic-function 'target-val :lambda-list '(m))
(cl:defmethod target-val ((m <forces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:target-val is deprecated.  Use sensors-msg:target instead.")
  (target m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <forces>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader sensors-msg:current-val is deprecated.  Use sensors-msg:current instead.")
  (current m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <forces>) ostream)
  "Serializes a message object of type '<forces>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'target))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'current))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <forces>) istream)
  "Deserializes a message object of type '<forces>"
  (cl:setf (cl:slot-value msg 'target) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'target)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'current) (cl:make-array 6))
  (cl:let ((vals (cl:slot-value msg 'current)))
    (cl:dotimes (i 6)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<forces>)))
  "Returns string type for a message object of type '<forces>"
  "sensors/forces")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'forces)))
  "Returns string type for a message object of type 'forces"
  "sensors/forces")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<forces>)))
  "Returns md5sum for a message object of type '<forces>"
  "7883231c3f2246aabc26c3b35f5b8006")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'forces)))
  "Returns md5sum for a message object of type 'forces"
  "7883231c3f2246aabc26c3b35f5b8006")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<forces>)))
  "Returns full string definition for message of type '<forces>"
  (cl:format cl:nil "float32[6] target~%float32[6] current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'forces)))
  "Returns full string definition for message of type 'forces"
  (cl:format cl:nil "float32[6] target~%float32[6] current~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <forces>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'target) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <forces>))
  "Converts a ROS message object to a list"
  (cl:list 'forces
    (cl:cons ':target (target msg))
    (cl:cons ':current (current msg))
))
